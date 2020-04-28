#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include "tf/tf.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "prism_explore");
  ros::NodeHandle n;

  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS costmap("explore_costmap", tf);

  costmap_2d::Costmap2D* master_map = costmap.getCostmap();

  unsigned int sizeX = master_map->getSizeInCellsX();
  unsigned int sizeY = master_map->getSizeInCellsY();
  unsigned int originX = master_map->getOriginX();
  unsigned int originY = master_map->getOriginY();
  unsigned int resolution = master_map->getResolution();

  ROS_INFO("size x: %d size y: %d", sizeX, sizeY);
  ROS_INFO("origin x: %d origin y: %d", originX, originY);
  ROS_INFO("resolution: %d", resolution);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
