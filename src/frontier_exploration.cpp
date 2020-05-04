#include "../include/frontier_exploration.h"
#include "ros/ros.h"
#include <Eigen/Dense>
#include <queue>

// Constructor
FrontierExplore::FrontierExplore(costmap_2d::Costmap2DROS* costmap2dROS, MoveBaseClient& acIn)
    : ac(acIn)
{
    costmap2dROS_ = costmap2dROS;
    costmap_ = costmap2dROS->getCostmap();
    ROS_INFO("cost_map global frame: %s", costmap2dROS_->getGlobalFrameID().c_str());
}

void FrontierExplore::moveToCell(int x, int y)
{
    // Create goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Find x and y relative to world
    double mx, my;
    this->costmap_->mapToWorld(x, y, mx, my);

    goal.target_pose.pose.position.x = mx;
    goal.target_pose.pose.position.y = my;

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

void FrontierExplore::frontierSearch()
{
    std::pair<int, int> pose = this->robotMapPos();
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
void FrontierExplore::printMapInfo()
{
    unsigned int sizeX = costmap_->getSizeInCellsX();
    unsigned int sizeY = costmap_->getSizeInCellsY();
    unsigned int originX = costmap_->getOriginX();
    unsigned int originY = costmap_->getOriginY();
    unsigned int resolution = costmap_->getResolution();

    ROS_INFO("size x: %d size y: %d", sizeX, sizeY);
    ROS_INFO("origin x: %d origin y: %d", originX, originY);
    ROS_INFO("resolution: %d", resolution);
}
