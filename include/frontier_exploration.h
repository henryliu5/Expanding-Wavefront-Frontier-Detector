#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <utility> 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class FrontierExplore {
public:
    FrontierExplore(costmap_2d::Costmap2DROS *costmap2dROS, MoveBaseClient &acIn);
    std::pair<int, int> frontierSearch();
    std::pair<int, int> getMapInfo();
    std::pair<int, int> robotMapPos();
    void moveToCell(int x, int y);
    void testCb(const ros::TimerEvent& e);

protected:
    costmap_2d::Costmap2DROS *costmap2dROS_;
    costmap_2d::Costmap2D *costmap_;
    MoveBaseClient &ac;
    int mapX;
    int mapY;
};

#endif