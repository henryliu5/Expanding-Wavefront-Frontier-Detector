#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <utility> 

class FrontierExplore {
public:
    FrontierExplore(costmap_2d::Costmap2DROS *costmap2dROS);
    void frontierSearch();
    void printMapInfo();
    std::pair<int, int> robotMapPos();

protected:
    costmap_2d::Costmap2DROS *costmap2dROS_;
    costmap_2d::Costmap2D *costmap_;
};

#endif