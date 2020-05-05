#include "../include/frontier_exploration.h"
#include "../include/marker.h"
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

    // Initialize marker publisher and wrapper for frontierExplore
    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>("test_marker", 1);
    CellMarker marker(&costmap, vis_pub);

    FrontierExplore frontierExplore(&costmap, ac, marker);
    std::pair<int, int> coords = frontierExplore.robotMapPos();


    // for(int i = 0; i < 800; i++){
    //   for(int j = 0; j < 300; j++){
    //     marker.addMarker(i, j);
    //   }
    // }
    visualization_msgs::MarkerArray marker_array_msg = marker.getMarkerArray();

    while (vis_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    // Test movement

    std::pair<int, int> pose = frontierExplore.robotMapPos();
    frontierExplore.moveToCell(pose.first, pose.second);


    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
