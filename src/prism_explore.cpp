#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include "tf/tf.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "prism_explore");
  ros::NodeHandle n;

  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS costmap("my_costmap", tf);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
