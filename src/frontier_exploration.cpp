#include "../include/frontier_exploration.h"
#include "ros/ros.h"
#include <Eigen/Dense>
#include <queue>

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using namespace std;

// Constructor
FrontierExplore::FrontierExplore(costmap_2d::Costmap2DROS* costmap2dROS, MoveBaseClient& acIn, CellMarker& markIn)
    : ac(acIn), marker(markIn)
{
    costmap2dROS_ = costmap2dROS;
    costmap_ = costmap2dROS->getCostmap();
    std::pair<int, int> dimensions = getMapInfo();
    mapX = dimensions.first;
    mapY = dimensions.second;
    std::vector<bool> temp (mapX * mapY, false);
    visited = temp;
    visitedFrontier = temp;

    ROS_INFO("cost_map global frame: %s", costmap2dROS_->getGlobalFrameID().c_str());
}

// Sends move command to navigate to specificed cell
// Will orient based on the start point
void FrontierExplore::moveToCell(int x, int y)
{
    // Create goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Find x and y relative to world
    double wx, wy;
    costmap_->mapToWorld(x, y, wx, wy);

    goal.target_pose.pose.position.x = wx;
    goal.target_pose.pose.position.y = wy;

    // Calculate orientation
    Eigen::Quaterniond orient;
    orient = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    // goal.target_pose.pose.orientation.x = orient.x();
    // goal.target_pose.pose.orientation.y = orient.y();
    // goal.target_pose.pose.orientation.z = orient.z();
    goal.target_pose.pose.orientation.w = 1;

    ROS_INFO("Sending goal to cell (%d, %d)", x, y);
    marker.addMarker(x, y, 2);

    ac.sendGoal(goal,
                boost::bind(&FrontierExplore::moveCb, this, _1),
                MoveBaseClient::SimpleActiveCallback());
}

// Callback for moveToCell, runs frontier search and calls moveToCell again
void FrontierExplore::moveCb(const actionlib::SimpleClientGoalState& state)
{
    
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    // We tried moving to this point -- regardless of success or failure remove it from the list of frontiers
    if(frontierList_.size() > 0){
        frontierList_.erase(frontierList_.begin());
        // Check that the frontiers are still frontiers;
         for(int i = 0; i < frontierList_.size(); i++){
             pair<int, int> cell = frontierList_[i];
             if(costmap_->getCost(cell.first, cell.second) == FREE_SPACE){
                 frontierList_.erase(frontierList_.begin() + i);
             }
         }
    } 
    if(state != actionlib::SimpleClientGoalState::SUCCEEDED){
        // ros::shutdown();
    }   
    frontierSearch();
    std::pair<int, int> frontier = frontierList_[0];
    ROS_INFO("moving to (%d, %d)", frontier.first, frontier.second);
    moveToCell(frontier.first, frontier.second);
}

// Search for frontiers and generate frontier list
void FrontierExplore::frontierSearch()
{
    std::vector<bool> temp (mapX * mapY, false);
    visited = temp;
    // Initialize queue and frontier list
    std::queue<std::pair<int, int> > q;

    // Enqueue initial pose
    // TODO Ensure first thing enqueued is always not visited and free space (in case robot missed goal)
    std::pair<int, int> pose = robotMapPos();
    if(!visited[costmap_->getIndex(pose.first, pose.second)]){
        q.push(pose);
    }

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
                        frontierList_.push_back(innerSearch(neighbor, visitedFrontier));
                    }
                }
            }
        }
    }
    ROS_INFO("search concluded, frontier list size: %d", (int)frontierList_.size());
    searching = false;
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
    marker.addMarker(centroid.first, centroid.second, 1);
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
