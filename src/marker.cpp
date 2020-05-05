#include "../include/marker.h"
#include "ros/ros.h"

// Constructor
CellMarker::CellMarker(costmap_2d::Costmap2DROS* costmap2dROS, ros::Publisher& vis_pub)
    : vis_pub_(vis_pub)
{
    costmap2dROS_ = costmap2dROS;
    costmap_ = costmap2dROS->getCostmap();
    ROS_INFO("cost_map global frame: %s", costmap2dROS_->getGlobalFrameID().c_str());
    size = 0;
}

void CellMarker::addMarker(int x, int y, int color)
{
    // Find x and y relative to world
    double wx, wy;
    costmap_->mapToWorld(x, y, wx, wy);

    tf::Stamped<tf::Pose> robotPose;
    costmap2dROS_->getRobotPose(robotPose);

    double worldPoseX = wx;
    double worldPoseY = wy;
    double worldPoseZ = robotPose.getOrigin().z();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = size;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = worldPoseX;
    marker.pose.position.y = worldPoseY;
    marker.pose.position.z = worldPoseZ;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    if (color == 1) {
        // RED
        marker.scale.x = .05;
        marker.scale.y = .05;
        marker.scale.z = .05;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else if (color == 2) {
        // BLUE
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = .05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    marker_array.markers.push_back(marker);

    vis_pub_.publish(marker_array);
    size++;
}

void CellMarker::removeMarker()
{
    marker_array.markers.pop_back();
    size--;
}

visualization_msgs::MarkerArray CellMarker::getMarkerArray()
{
    return marker_array;
}
