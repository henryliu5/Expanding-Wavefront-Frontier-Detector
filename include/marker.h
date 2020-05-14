#ifndef CELL_MARKER_H
#define CELL_MARKER_H

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <utility> 
#include <map>


class CellMarker {
public:
    CellMarker(costmap_2d::Costmap2DROS *costmap2dROS, ros::Publisher& vis_pub);
    void addMarker(int x, int y, int color);
    void removeMarker(int x, int y);
    visualization_msgs::MarkerArray getMarkerArray();

protected:
    visualization_msgs::MarkerArray marker_array;
    costmap_2d::Costmap2DROS *costmap2dROS_;
    costmap_2d::Costmap2D *costmap_;
    ros::Publisher& vis_pub_;
    int uniqueId;
    std::map<std::pair<int, int>, int> markerMap;
};

#endif