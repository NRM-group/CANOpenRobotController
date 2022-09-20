#ifndef __EXO_ROBOT_HPP
#define __EXO_ROBOT_HPP

/* Butterworth filter order */
#define FILTER_ORDER 2

/* STL */
#include <map>
#include <chrono>
#include <memory>
#include <string>
/* Libraries */
#include <Eigen/Dense>
#include "LookupTable.hpp"
#include "controller.hpp"
/* CORC interfaces */
#include "X2Robot.h"
/* ROS2 client library C++ */
#include "rclcpp/rclcpp.hpp"
/* Standard interfaces */
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
/* Exo interfaces */
#include "exo_msgs/msg/dev_toggle.hpp"
#include "exo_msgs/msg/endpoint.hpp"
#include "exo_msgs/msg/external_parameter.hpp"
#include "exo_msgs/msg/friction_parameter.hpp"
#include "exo_msgs/msg/gait_parameter.hpp"
#include "exo_msgs/msg/patient_parameter.hpp"
#include "exo_msgs/msg/pd_parameter.hpp"
#include "exo_msgs/msg/sit_to_stand_parameter.hpp"
#include "exo_msgs/msg/user_command.hpp"
/* ROS interface namespaces */
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace exo_msgs::msg;
using Float = Float64;
using FloatArray = Float64MultiArray;
/* ROS callback */
using std::placeholders::_1;

class ExoRobot : X2Robot, rclcpp::Node
{
public:
    ExoRobot(const std::string &__name);
    ~ExoRobot();

private: // ROS publish methods
    void publish_joint_state();
    void publish_joint_reference();
    void publish_strain_gauge();

private: // ROS subscription callbacks
    void dev_toggle_callback(const DevToggle::SharedPtr msg);
    void external_parameter_callback(const ExternalParameter::SharedPtr msg);
    void friction_parameter_callback(const FrictionParameter::SharedPtr msg);
    void gait_parameter_callback(const GaitParameter::SharedPtr msg);
    void maximum_torque_callback(const Double::SharedPtr msg);
    void patient_parameter_callback(const PatientParameter::SharedPtr msg);
    void pd_parameter_callback(const PDParameter::SharedPtr msg);
    void sit_to_stand_parameter_callback(const SitToStandParameter::SharedPtr msg);
    void user_command_callback(const UserCommand::SharedPtr msg);

protected: // ROS-LabVIEW message copies
    DevToggle _DevToggle;
    GaitParameter _GaitParameter;
    PatientParameter _PatientParameter;
    SitToStandParameter _SitToStandParameter;
    UserCommand _UserCommand;

protected: // Controllers
    ctrl::Butterworth<double, X2_NUM_JOINTS, FILTER_ORDER> _CtrlButterStrainGauge;
    ctrl::ExternalController<double, X2_NUM_JOINTS> _CtrlExternal;
    ctrl::FrictionController<double, X2_NUM_JOINTS> _CtrlFriction;
    ctrl::GravityController<double, X2_NUM_JOINTS> _CtrlGravity;
    ctrl::PDController<double, X2_NUM_JOINTS> _CtrlPD;
    ctrl::TorqueController<double, X2_NUM_JOINTS> _CtrlTorque;

private: // ROS publishers
    rclcpp::Publisher<JointState>::SharedPtr _PubJointState;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubJointReference;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubStrainGauge;

private: // ROS subscriptions
    rclcpp::Subscription<DevToggle>::SharedPtr _SubDevToggle;
    rclcpp::Subscription<ExternalParameter>::SharedPtr _SubExternalParameter;
    rclcpp::Subscription<FrictionParameter>::SharedPtr _SubFrictionParameter;
    rclcpp::Subscription<GaitParameter>::SharedPtr _SubGaitParameter;
    rclcpp::Subscription<Double>::SharedPtr _SubMaximumTorque;
    rclcpp::Subscription<PatientParameter>::SharedPtr _SubPatientParameter;
    rclcpp::Subscription<PDParameter>::SharedPtr _SubPDParameter;
    rclcpp::Subscription<SitToStandParameter>::SharedPtr _SubSitToStandParameter;
    rclcpp::Subscription<UserCommand>::SharedPtr _SubUserCommand
};

#endif//__EXO_ROBOT_HPP