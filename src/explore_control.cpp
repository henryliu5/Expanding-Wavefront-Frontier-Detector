#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "prism_explore_control");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("/prism_explore_node/control", 1000);

    char input;
    bool on = false;
    while (true) {
        std_msgs::String msg;
        std::stringstream ss;

        ROS_INFO("Press any key to toggle exploration");
        std::cin >> input;
        on = !on;
        if (on) {
            ss << "run";
        } else {
            ss << "stop";
        }

        msg.data = ss.str();
        pub.publish(msg);
    }

    return 0;
}
