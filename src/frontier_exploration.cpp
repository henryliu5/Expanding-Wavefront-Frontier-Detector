#include "../include/frontier_exploration.h"
#include "ros/ros.h"
#include <Eigen/Dense>
#include <queue>

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using namespace std;

// Constructor
FrontierExplore::FrontierExplore(costmap_2d::Costmap2DROS* costmap2dROS, MoveBaseClient& acIn)
    : ac(acIn)
{
    costmap2dROS_ = costmap2dROS;
    costmap_ = costmap2dROS->getCostmap();
    std::pair<int, int> dimensions = getMapInfo();
    mapX = dimensions.first;
    mapY = dimensions.second;
    // for(int x = 0; x < mapX; x++){
    //     for(int y = 0; y < mapY; y++){
    //         costmap_->setCost(x, y, -1);
    //     }
    // }

    ROS_INFO("cost_map global frame: %s", costmap2dROS_->getGlobalFrameID().c_str());
}

void FrontierExplore::testCb(const ros::TimerEvent& e)
{
    std::pair<int, int> frontier = frontierSearch()[0];
    ROS_INFO("moving to (%d, %d)", frontier.first, frontier.second);
    moveToCell(frontier.first, frontier.second);
}

// Sends move command to navigate to specificed cell
// Will orient based on the start point
void FrontierExplore::moveToCell(int x, int y)
{
    // Create goa
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Find x and y relative to world
    double mx, my;
    costmap_->mapToWorld(x, y, mx, my);

    goal.target_pose.pose.position.x = mx;
    goal.target_pose.pose.position.y = my;

    // Calculate orientation
    Eigen::Quaterniond orient;
    orient = Eigen::AngleAxisd(-atan(-my / mx), Eigen::Vector3d::UnitZ());
    goal.target_pose.pose.orientation.x = orient.x();
    goal.target_pose.pose.orientation.y = orient.y();
    goal.target_pose.pose.orientation.z = orient.z();
    goal.target_pose.pose.orientation.w = orient.w();

    ROS_INFO("Sending goal to cell (%d, %d)", x, y);
    ac.sendGoal(goal);

    //ac.waitForResult();
}

// Search for frontiers and generate frontier list
vector<pair<int, int> > FrontierExplore::frontierSearch()
{
    // Initialize queue and frontier list
    std::queue<std::pair<int, int> > q;
    std::vector<std::pair<int, int> > result;

    // Shorthand to initialize 2d matrix to all false
    std::vector<bool> visited(mapX * mapY, false);
    std::vector<bool> visitedFrontier(mapX * mapY, false);

    // Enqueue initial pose
    std::pair<int, int> pose = robotMapPos();
    q.push(pose);

    ROS_INFO("starting search");
    while (!q.empty()) {

        std::pair<int, int> start = q.front();
        q.pop();

        bool hasFreeSpaceNeighbor = false;
        // Iterate over neighbors
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                unsigned int x = start.first + dx;
                unsigned int y = start.second + dy;

                pair<int, int> neighbor(x, y);
                // Check bounds for neighbor
                if (x >= 0 && y >= 0 && x < mapX && y < mapY && !visited[costmap_->getIndex(x, y)]) {
                    unsigned char cost = costmap_->getCost(x, y);
                    ROS_INFO("checking cell (%d, %d), cost is: %d", x, y, cost);
                    // Check what the neighbor is
                    // If the neighbor is free (not visited checked already) then enqueue
                    if (cost == FREE_SPACE) {
                        hasFreeSpaceNeighbor = true;
                        q.push(neighbor);
                        visited[costmap_->getIndex(x, y)] = true;
                    } else if(isNewFrontier(neighbor, visitedFrontier)){
                        visitedFrontier[costmap_->getIndex(x, y)] = true;
                        result.push_back(innerSearch(neighbor, visitedFrontier));
                    }
                }
            }
        }
    }
    ROS_INFO("search concluded, found %d frontiers", (int)result.size());
    // for(std::pair<int, int> x: result){
    //     ROS_INFO("frontiers found: (%d, %d)", x.first, x.second);
    // }
    return result;
}

pair<int, int> FrontierExplore::innerSearch(pair<int, int> frontier, vector<bool>& visitedFrontier)
{
    queue< pair<int, int> > queueF;
    // Keep track of averages for this group to calculate centroid
    int sumX = frontier.first;
    int sumY = frontier.second;
    int thisFrontierSize = 1;
    queueF.push(frontier);

    while(!queueF.empty()){
        std::pair<int, int> start = queueF.front();
        queueF.pop();

        // Iterate over neighbors
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                unsigned int x = start.first + dx;
                unsigned int y = start.second + dy;

                pair<int, int> neighbor(x, y);
                // Check bounds for neighbor
                if (x >= 0 && y >= 0 && x < mapX && y < mapY) {
                    if(isNewFrontier(neighbor, visitedFrontier)){
                        visitedFrontier[costmap_->getIndex(x, y)] = true;
                        sumX += x;
                        sumY += y;
                        thisFrontierSize++;
                        queueF.push(neighbor);
                    }
                }
            }
        }
    }
    ROS_INFO("Grouped frontier points - total size: %d", thisFrontierSize);
    // Return centroid, allowing for int rounding to fit on cell
    pair<int, int> centroid (sumX / thisFrontierSize, sumY / thisFrontierSize);
    return centroid;
}

bool FrontierExplore::isNewFrontier(pair<int, int> neighbor, vector<bool>& visitedFrontier)
{
    // To be a new frontier point this cell must be unknown and not be part of an existing frontier
    if(costmap_->getCost(neighbor.first, neighbor.second) != NO_INFORMATION || visitedFrontier[costmap_->getIndex(neighbor.first, neighbor.second)]){
        return false;
    }

    // Iterate over neighbors of this cell
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            unsigned int x = neighbor.first + dx;
            unsigned int y = neighbor.second + dy;

            pair<int, int> neighbor(x, y);
            // Check bounds for neighbor
            if (x >= 0 && y >= 0 && x < mapX && y < mapY) {
                unsigned char cost = costmap_->getCost(x, y);
                // Check what the neighbor is
                if (cost == FREE_SPACE) {
                    return true;
                }
            }
        }
    }
    return false;
}

// Returns pair of ints representing x and y coordinates of the robot's pose on the map
std::pair<int, int> FrontierExplore::robotMapPos()
{
    // Get pose of robot relative to /map
    tf::Stamped<tf::Pose> robotPose;
    costmap2dROS_->getRobotPose(robotPose);
    double worldPoseX = robotPose.getOrigin().x();
    double worldPoseY = robotPose.getOrigin().y();

    ROS_INFO("world robot pose x: %f", worldPoseX);
    ROS_INFO("world robot pose y: %f", worldPoseY);

    // Convert pose to map cells
    unsigned int mapCellX;
    unsigned int mapCellY;
    bool withinBounds = costmap_->worldToMap(worldPoseX, worldPoseY, mapCellX, mapCellY);
    ROS_INFO("world robot pose within bounds: %d", withinBounds);
    ROS_INFO("map robot pose x: %d", mapCellX);
    ROS_INFO("map robot pose y: %d", mapCellY);

    std::pair<int, int> result(mapCellX, mapCellY);
    return result;
}

// Displays info about map size, location, and resolution
std::pair<int, int> FrontierExplore::getMapInfo()
{
    unsigned int sizeX = costmap_->getSizeInCellsX();
    unsigned int sizeY = costmap_->getSizeInCellsY();
    unsigned int originX = costmap_->getOriginX();
    unsigned int originY = costmap_->getOriginY();
    unsigned int resolution = costmap_->getResolution();

    ROS_INFO("size x: %d size y: %d", sizeX, sizeY);
    ROS_INFO("origin x: %d origin y: %d", originX, originY);
    ROS_INFO("resolution: %d", resolution);
    std::pair<int, int> result(sizeX, sizeY);
    return result;
}
