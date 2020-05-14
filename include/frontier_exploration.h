#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include "./frontier.h"
#include "./marker.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <utility>
#include <vector>
#include "std_msgs/String.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class FrontierExplore {
public:
    FrontierExplore(costmap_2d::Costmap2DROS* costmap2dROS, MoveBaseClient& acIn, CellMarker& markIn, ros::NodeHandle &n);
    void frontierSearch(std::pair<int, int> start);
    std::pair<int, int> innerSearch(std::pair<int, int> frontier, std::vector<bool>& visitedFrontier);
    std::pair<int, int> getMapInfo();
    std::pair<int, int> robotMapPos();
    void moveToCell(int x, int y);
    bool isNewFrontier(std::pair<int, int> start, std::vector<bool>& visited);
    void moveCb(const actionlib::SimpleClientGoalState& state);
    void stateCallback(const std_msgs::String::ConstPtr& msg);

protected:
    costmap_2d::Costmap2DROS* costmap2dROS_;
    costmap_2d::Costmap2D* costmap_;
    MoveBaseClient& ac;
    CellMarker& marker;
    int mapX;
    int mapY;
    std::vector<bool> visited;
    std::vector<bool> visitedFrontier;
    std::vector< std::pair<int, int> > frontierList_;
    std::vector< std::pair<int, int> > frontierHistory_;
    bool state;
    std::pair<int, int> lastGoal;
    ros::Subscriber sub;
};

#endif