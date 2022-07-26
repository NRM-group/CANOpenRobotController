#ifndef EXO_STATE_HPP
#define EXO_STATE_HPP

/* Compile-time parameters */
#define FILTER_ORDER        2
#define FILTER_COEFF_A      { 1, 2, 1 }
#define FILTER_COEFF_B      { 124.059, 214.679, -94.619 } 
/***************************/

/* CORC interface */
#include "State.h"
#include "X2Robot.h"
/* Standard interfaces */
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
/* Exo interfaces */
#include "exo_msgs/msg/command.hpp"
#include "exo_msgs/msg/corc.hpp"
#include "exo_msgs/msg/enable.hpp"
#include "exo_msgs/msg/endpoint.hpp"
#include "exo_msgs/msg/external.hpp"
#include "exo_msgs/msg/friction.hpp"
#include "exo_msgs/msg/output.hpp"
#include "exo_msgs/msg/pd.hpp"
#include "exo_msgs/msg/sit_to_stand.hpp"
#include "exo_msgs/msg/user.hpp"
#include "exo_msgs/msg/walk.hpp"
/* ROS2 client library C++ */
#include "rclcpp/rclcpp.hpp"
/* Libraries */
//#include "LookupTable.hpp"
#include "controller.hpp"
/* ROS interface aliases */
using Float = std_msgs::msg::Float64;
using FloatArray = std_msgs::msg::Float64MultiArray;
using JointState = sensor_msgs::msg::JointState;
using Corc = exo_msgs::msg::Corc;
using External = exo_msgs::msg::External;
using Friction = exo_msgs::msg::Friction;
using Output = exo_msgs::msg::Output;
using PD = exo_msgs::msg::PD;
/* ROS callback */
using std::placeholders::_1;

///////////////////////////////////////////////////////////////////////////////
class RunState : public State, public rclcpp::Node
{
public: // Constructors / Destructors
    explicit RunState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot);
    RunState(const RunState &) = delete;
    RunState(RunState &&) = delete;
    ~RunState();

public: // Override methods
    void entry();
    void during();
    void exit();

private: // Helper methods
    void calculate_torque();

private: // ROS methods
    void publish_joint_state();
    void publish_output();
    void publish_strain_gauge();
    void corc_callback(const Corc::SharedPtr msg);
    void external_callback(const External::SharedPtr msg);
    void friction_callback(const Friction::SharedPtr msg);
    void pd_callback(const PD::SharedPtr msg);

private: // Local parameters
    Eigen::Vector4d _TorqueOutput;
    double _TorqueLimit;
    std::array<double, X2_NUM_JOINTS> _StrainGaugeOffset;

private: // Controllers
    ctrl::PDController<double, X2_NUM_JOINTS> _CtrlPD;
    ctrl::ExternalController<double, X2_NUM_JOINTS> _CtrlExternal;
    ctrl::FrictionController<double, X2_NUM_JOINTS> _CtrlFriction;
    ctrl::GravityController<double, X2_NUM_JOINTS> _CtrlGravity;
    ctrl::TorqueController<double, X2_NUM_JOINTS> _CtrlTorque;
    ctrl::Butterworth<double, X2_NUM_JOINTS, FILTER_ORDER> _CtrlButterStrainGauge;

private: // ROS publishers / subscribers
    std::shared_ptr<X2Robot> _Robot;
    rclcpp::Publisher<JointState>::SharedPtr _PubJointState;
    rclcpp::Publisher<Output>::SharedPtr _PubOutput;
    rclcpp::Subscription<Corc>::SharedPtr _SubCorc;
    rclcpp::Subscription<External>::SharedPtr _SubExternal;
    rclcpp::Subscription<Friction>::SharedPtr _SubFriction;
    rclcpp::Subscription<PD>::SharedPtr _SubPD;

private: // FIXME: temporary
    rclcpp::Publisher<FloatArray>::SharedPtr _PubStrainGauge;
    rclcpp::Subscription<FloatArray>::SharedPtr _SubStrainGauge;
    void strain_gauge_callback(const FloatArray::SharedPtr msg);
    double scale[8];
    double sg[4];
};

///////////////////////////////////////////////////////////////////////////////
class OffState : public State
{
public: // Constructors / Destructors
    explicit OffState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot);
    OffState(const OffState &) = delete;
    OffState(OffState &&) = delete;
    ~OffState();

public: // Override methods
    void entry();
    void during();
    void exit();

private: // Robot access
    std::shared_ptr<X2Robot> _Robot;
};

#endif