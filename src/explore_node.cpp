#include "../include/frontier_exploration.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include <costmap_2d/costmap_2d_ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "prism_explore");
    ros::NodeHandle n;

    // Establish TF listener for costmap
    tf::TransformListener tf(ros::Duration(10));

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("/move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Initialize Costmap2DROS - wrapper for accessing costmap in C++
    costmap_2d::Costmap2DROS costmap("explore_costmap", tf);

    FrontierExplore frontierExplore(&costmap, ac);
    std::pair<int, int> coords = frontierExplore.robotMapPos();

    // Test movement
    //frontierExplore.moveToCell(798, 200);

    // std::pair<int, int> frontier = frontierExplore.frontierSearch();
    // ROS_INFO("moving to (%d, %d)", frontier.first, frontier.second);
    // frontierExplore.moveToCell(frontier.first, frontier.second);
    ros::Timer timer = n.createTimer(ros::Duration(30), &FrontierExplore::testCb, &frontierExplore);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
