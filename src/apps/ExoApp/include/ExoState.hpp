#ifndef EXO_STATE_HPP
#define EXO_STATE_HPP

/***************************/
/* Compile-time parameters */
#define FILTER_ORDER    2
/***************************/

/* CORC interface */
#include "State.h"
#include "X2Robot.h"
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

///////////////////////////////////////////////////////////////////////////////
// Run state
///////////////////////////////////////////////////////////////////////////////
class RunState : public State, public rclcpp::Node
{
public: // Constructors / Destructors
    explicit RunState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot);
    RunState(const RunState &) = delete;
    RunState(RunState &&) = delete;
    ~RunState();

public: // Override methods
    void entry() override;
    void during() override;
    void exit() override;

private: // Helper methods
    void limit_torque();

private: // ROS publish methods
    void publish_joint_state();
    void publish_joint_reference();
    void publish_strain_gauge();

private: // ROS subscribe methods
    void dev_toggle_callback(const DevToggle::SharedPtr msg);
    void external_parameter_callback(const ExternalParameter::SharedPtr msg);
    void friction_parameter_callback(const FrictionParameter::SharedPtr msg);
    void gait_parameter_callback(const GaitParameter::SharedPtr msg);
    void maximum_torque_callback(const Double::SharedPtr msg);
    void patient_parameter_callback(const PatientParameter::SharedPtr msg);
    void pd_parameter_callback(const PDParameter::SharedPtr msg);
    void sit_to_stand_parameter_callback(const SitToStandParameter::SharedPtr msg);
    void user_command_callback(const UserCommand::SharedPtr msg);

private: // Local parameters
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
    std::shared_ptr<X2Robot> _Robot;
    rclcpp::Publisher<JointState>::SharedPtr _PubJointState;
    rclcpp::Publisher<DoubleArray>::SharedPtr _PubJointReference;
    rclcpp::Publisher<DoubleArray>::SharedPtr _PubStrainGauge;

private: // ROS subscribers
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

///////////////////////////////////////////////////////////////////////////////
// Off state
///////////////////////////////////////////////////////////////////////////////
class OffState : public State
{
public: // Constructors / Destructors
    explicit OffState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot);
    OffState(const OffState &) = delete;
    OffState(OffState &&) = delete;
    ~OffState();

public: // Override methods
    void entry() override;
    void during() override;
    void exit() override;

private: // Robot access
    std::shared_ptr<X2Robot> _Robot;
};

#endif
