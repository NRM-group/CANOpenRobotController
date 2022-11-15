#ifndef __EXO_NODE_HPP__
#define __EXO_NODE_HPP__

/* STL */
#include <map>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <ctime>
#include <cstddef>
/* CORC interface */
#include "X2Robot.h"
/* ROS2 client library C++ */
#include <rclcpp/rclcpp.hpp>
/* Standard interfaces */
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
/* ROS interface namespaces */
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
/* ROS callback */
using std::placeholders::_1;

class ExoNode : rclcpp::Node
{
public:
    using Float = Float64;
    using FloatArray = Float64MultiArray;

public: // Constructor
    explicit ExoNode(std::shared_ptr<X2Robot> robot);

public: // ROS method visibility modifiers
    void ros_declare(const std::vector<std::string> &names);
    void ros_parameter(const std::string &name, std::vector<double> &val);
    void get_exo_file(std::string &path);
    void get_gait_file(std::string &path);
    void get_affc_file(std::string &path);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_interface();

public: // ROS publish methods
    void publish_joint_reference(const std::vector<double> &val);
    void publish_joint_vel_reference(const std::vector<double> &val);
    void publish_joint_accel_reference(const std::vector<double> &val);
    void publish_joint_state();
    void publish_affc_torque(const std::vector<double> &val);
    void publish_affc_known_parameters(const std::vector<double> &leftUpper, 
                                       const std::vector<double> &rightUpper,
                                       const std::vector<double> &leftLower,
                                       const std::vector<double> &rightLower);
    void publish_affc_unknown_parameters(const std::vector<double> &left,
                                         const std::vector<double> &right);
    void publish_affc_tracking_error(const std::vector<double> &val);
    void publish_affc_sgd_error(const std::vector<double> &val);

public:
    const double get_torque_limit() const;

private: // ROS subscription callbacks
    void torque_limit_callback(const Float::SharedPtr msg);

private: // Internal robot reference
    const std::shared_ptr<X2Robot> _Robot;

private: // ROS-LabVIEW message copies
    double _TorqueLimit;

private: // ROS publishers
    rclcpp::Publisher<JointState>::SharedPtr _PubJointState;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubJointReference;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubJointVelReference;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubJointAccelReference;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubAffcTorque;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubAffcLeftUpperKnownParameters;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubAffcLeftLowerKnownParameters;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubAffcRightUpperKnownParameters;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubAffcRightLowerKnownParameters;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubAffcLeftUnknownParameters;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubAffcRightUnknownParameters;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubAffcTrackingError;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubAffcSGDError;

private: // ROS subscriptions
    rclcpp::Subscription<Float>::SharedPtr _SubTorqueLimit;
};

#endif//__EXO_NODE_HPP__