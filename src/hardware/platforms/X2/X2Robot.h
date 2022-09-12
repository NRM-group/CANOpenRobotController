#ifndef X2ROBOT_H
#define X2ROBOT_H

/***************************/
/* Compile-time parameters */
#define FILTER_ORDER    2
/***************************/

/* STL */
#include <map>
#include <chrono>
/* Dependency */
#include <Eigen/Dense>
/* CORC interfaces */
#include "Robot.h"
#include "X2Joint.h"
#include "CopleyDrive.h"
#include "FourierForceSensor.h"
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
/* ROS2 client library C++ */
#include "rclcpp/rclcpp.hpp"
/* Libraries */
#include "LookupTable.hpp"
#include "controller.hpp"
/* ROS interface namespaces */
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace exo_msgs::msg;
using Double = Float64;
using DoubleArray = Float64MultiArray;
/* ROS callback */
using std::placeholders::_1;


#define X2_NUM_JOINTS 4

JointDrivePairs HipJDP { 250880, 0, 1.57, 0 };

JointDrivePairs KneeJDP { 0, 250880, 0, -1.57 };

class X2Robot : public Robot, public rclcpp::Node
{
public: // Constructors / destructors
    X2Robot();
    ~X2Robot();

public: // Robot interface
    void kill_robot();
    void update_robot();
    void set_position(const Eigen::Vector4d &pos);
    void set_velocity(const Eigen::Vector4d &vel);
    void set_torque(const Eigen::Vector4d &tor);
    bool set_control(ControlMode mode);

public: // Behaviour methods
    void update_node();
    void start_trajectory();
    void execute();
    void calibrate();

private: // Initializers
    void init();
    bool initialiseInputs() override;
    bool initialiseJoints() override;
    bool initialiseNetwork() override;

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

private: // Robot parameters
    motorProfile _PosMotorProfile;
    motorProfile _VelMotorProfile;
    ControlMode _ControlMode;
    std::array<std::shared_ptr<Drive>, X2_NUM_JOINTS> _MotorDrives;
    std::array<std::shared_ptr<FourierForceSensor>, X2_NUM_JOINTS> _ForceSensors;
    double _MaxTorque, _MaxVelocity, _MaxHip, _MinHip, _MaxKnee, _MinKnee;
    Eigen::Vector4d _DesiredPositions;
    double _TorqueLimit;
    Eigen::Vector4d _TorqueOutput;
    Eigen::Vector4d _StrainGauge;
    Eigen::Vector4d _StrainGaugeOffset;
    Eigen::Vector4d _StrainGaugeScale;
    LookupTable<double, X2_NUM_JOINTS> _LookupTable;

private: // ROS-LabVIEW message copies
    DevToggle _DevToggle;
    GaitParameter _GaitParameter;
    PatientParameter _PatientParameter;
    SitToStandParameter _SitToStandParameter;
    UserCommand _UserCommand;

private: // Controllers
    ctrl::Butterworth<double, X2_NUM_JOINTS, FILTER_ORDER> _CtrlButterStrainGauge;
    ctrl::ExternalController<double, X2_NUM_JOINTS> _CtrlExternal;
    ctrl::FrictionController<double, X2_NUM_JOINTS> _CtrlFriction;
    ctrl::GravityController<double, X2_NUM_JOINTS> _CtrlGravity;
    ctrl::PDController<double, X2_NUM_JOINTS> _CtrlPD;
    ctrl::TorqueController<double, X2_NUM_JOINTS> _CtrlTorque;

private: // ROS publishers
    rclcpp::Publisher<JointState>::SharedPtr _PubJointState;
    rclcpp::Publisher<DoubleArray>::SharedPtr _PubJointReference;
    rclcpp::Publisher<DoubleArray>::SharedPtr _PubStrainGauge;

private: // ROS subscriptions
    rclcpp::Subscription<DevToggle>::SharedPtr _SubDevToggle;
    rclcpp::Subscription<ExternalParameter>::SharedPtr _SubExternalParameter;
    rclcpp::Subscription<FrictionParameter>::SharedPtr _SubFrictionParameter;
    rclcpp::Subscription<GaitParameter>::SharedPtr _SubGaitParameter;
    rclcpp::Subscription<Double>::SharedPtr _SubMaximumTorque;
    rclcpp::Subscription<PatientParameter>::SharedPtr _SubPatientParameter;
    rclcpp::Subscription<PDParameter>::SharedPtr _SubPDParameter;
    rclcpp::Subscription<SitToStandParameter>::SharedPtr _SubSitToStandParameter;
    rclcpp::Subscription<UserCommand>::SharedPtr _SubUserCommand;
};

#endif