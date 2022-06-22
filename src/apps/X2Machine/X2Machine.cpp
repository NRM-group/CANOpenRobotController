#include "X2Machine.h"

#define OWNER ((X2Machine *)owner)

X2Machine::X2Machine(int argc, char** argv, const float updateT) {

    // specify custom SIGINT handler is going to be used
    rclcpp::InitOptions rosInit = rclcpp::InitOptions();
    rosInit.shutdown_on_sigint = false;

    // initalise ROS2 node
    rclcpp::init(argc, argv, rosInit);
    node = rclcpp::Node::make_shared("x2");

    // get robot name from node name and remove '/'
    robotName_ = node->get_name();
    robotName_.erase(0, 1);

#ifdef SIM
    robot_ = new X2Robot(nodeHandle, updateT, robotName_);
#else
    robot_ = new X2Robot(updateT, robotName_);
#endif

    // Create PRE-DESIGNED State Machine events and state objects
    startExo = new StartExo(this);
}