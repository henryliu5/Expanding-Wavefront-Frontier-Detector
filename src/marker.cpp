#include "../include/marker.h"
#include "ros/ros.h"

// Constructor
Marker::Marker() {
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_array_msg.markers[0] = marker;
    size = 0;
}

void Marker::addMarker(Pose relPose){
    marker.pose.position.x = relPose.x;
    marker.pose.position.y = relPose.y;
    marker.pose.position.z = relPose.z;
    marker.pose.orientation.x = relPose.orientation.x;
    marker.pose.orientation.y = relPose.orientation.y;
    marker.pose.orientation.z = relPose.orientation.z;
    marker.pose.orientation.w = relPose.orientation.w;
    marker_array_msg.markers[size] = marker;
    size++;
}

