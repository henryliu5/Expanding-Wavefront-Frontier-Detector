#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include "tf/tf.h"
#include "../include/frontier_exploration.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prism_explore");
    ros::NodeHandle n;

    // Establish TF listener for costmap
    tf::TransformListener tf(ros::Duration(10));
    // Initialize Costmap2DROS - wrapper for accessing costmap in C++
    costmap_2d::Costmap2DROS costmap("explore_costmap", tf);
    
    FrontierExplore frontierExplore(&costmap);
    std::pair<int, int> coords = frontierExplore.robotMapPos();

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
