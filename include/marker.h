#ifndef MARKER_H
#define MARKER_H

#include "ros/ros.h"
#include "tf/tf.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class Marker {
public:
    Marker();
    void addMarker();
    void removeMarker();
protected:
    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::Marker marker;
    int size;
};

#endif