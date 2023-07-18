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
/* Exo interfaces */
#include "exo_msgs/msg/dev_toggle.hpp"
#include "exo_msgs/msg/endpoint.hpp"
#include "exo_msgs/msg/external_parameter.hpp"
#include "exo_msgs/msg/friction_parameter.hpp"
#include "exo_msgs/msg/gait_parameter.hpp"
#include "exo_msgs/msg/heart_beat.hpp"
#include "exo_msgs/msg/patient_parameter.hpp"
#include "exo_msgs/msg/pd_parameter.hpp"
#include "exo_msgs/msg/sit_to_stand_parameter.hpp"
#include "exo_msgs/msg/torque_parameter.hpp"
#include "exo_msgs/msg/user_command.hpp"
/* ROS interface namespaces */
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace exo_msgs::msg;
/* ROS callback */
using std::placeholders::_1;

/**
 * @brief ExoNode is a ROS node used to publish (StateMachine/Exo-msgs) and subcribe to (LabView-msgs) topics for the ROS computer.
 */
class ExoNode : rclcpp::Node
{
public:
    using Float = Float64;
    using FloatArray = Float64MultiArray;
    enum HeartBeatStatus
    {
        OK, DEAD, ESTOP, RESTART, INTERNAL
    };

public: // Constructor
    explicit ExoNode(std::shared_ptr<X2Robot> robot);

public: // State machine event flags
    bool overwrite_save();
    bool save_error();
    bool is_saved();
    bool ok();

public: // ROS method visibility modifiers
    void ros_declare(const std::vector<std::string> &names);
    void ros_parameter(const std::string &name, std::vector<double> &val);
    void get_exo_file(std::string &path);
    void get_gait_file(std::string &path);
    void set_save_error(bool val);
    void set_is_saved(bool val);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_interface();

public: // ROS publish methods
    void publish_heart_beat();
    void publish_joint_reference(const std::vector<double> &val);   
    void publish_joint_state(); 
    void publish_strain_gauge();
    void publish_gait_index(double gaitIndex);
    void publish_error(const Eigen::Matrix<double, 4, 1> &error);
    void publish_der_error(const Eigen::Matrix<double, 4, 1> &derError);

public: // Get ROS-LabVIEW message copies
    DevToggle & get_dev_toggle();
    ExternalParameter & get_external_parameter();
    FrictionParameter & get_friction_parameter();
    GaitParameter & get_gait_parameter();
    HeartBeat & get_heart_beat();
    PatientParameter & get_patient_parameter();
    PDParameter & get_pd_parameter();
    SitToStandParameter & get_sit_to_stand_parameter();
    double get_torque_limit();
    TorqueParameter & get_torque_parameter();
    UserCommand & get_user_command();

private: // ROS subscription callbacks
    void dev_toggle_callback(const DevToggle::SharedPtr msg);
    void external_parameter_callback(const ExternalParameter::SharedPtr msg);
    void friction_parameter_callback(const FrictionParameter::SharedPtr msg);
    void gait_parameter_callback(const GaitParameter::SharedPtr msg);
    void heart_beat_callback(const HeartBeat::SharedPtr msg);
    void torque_limit_callback(const Float::SharedPtr msg);
    void patient_parameter_callback(const PatientParameter::SharedPtr msg);
    void pd_parameter_callback(const PDParameter::SharedPtr msg);
    void sit_to_stand_parameter_callback(const SitToStandParameter::SharedPtr msg);
    void torque_parameter_callback(const TorqueParameter::SharedPtr msg);
    void user_command_callback(const UserCommand::SharedPtr msg);

private: // Internal robot reference
    const std::shared_ptr<X2Robot> _Robot;
    std::chrono::_V2::system_clock::time_point _Midnight;
    bool _SaveError;
    bool _IsSaved;

private: // ROS-LabVIEW message copies
    DevToggle _DevToggle;
    ExternalParameter _ExternalParameter;
    FrictionParameter _FrictionParameter;
    GaitParameter _GaitParameter;
    HeartBeat _HeartBeat;
    PatientParameter _PatientParameter;
    PDParameter _PDParameter;
    SitToStandParameter _SitToStandParameter;
    double _TorqueLimit;
    TorqueParameter _TorqueParameter;
    UserCommand _UserCommand;

private: // ROS publishers
    rclcpp::Publisher<HeartBeat>::SharedPtr _PubHeartBeat;
    rclcpp::Publisher<JointState>::SharedPtr _PubJointState;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubJointReference;
    rclcpp::Publisher<FloatArray>::SharedPtr _PubStrainGauge;
    rclcpp::Publisher<Float>::SharedPtr _PubErrorLH;
    rclcpp::Publisher<Float>::SharedPtr _PubErrorLK;
    rclcpp::Publisher<Float>::SharedPtr _PubErrorRH;
    rclcpp::Publisher<Float>::SharedPtr _PubErrorRK;
    rclcpp::Publisher<Float>::SharedPtr _PubDerErrorLH;
    rclcpp::Publisher<Float>::SharedPtr _PubDerErrorLK;
    rclcpp::Publisher<Float>::SharedPtr _PubDerErrorRH;
    rclcpp::Publisher<Float>::SharedPtr _PubDerErrorRK;
    rclcpp::Publisher<Float>::SharedPtr _PubGaitIndex;
    rclcpp::Publisher<Float>::SharedPtr _PubLH_Ref;
    rclcpp::Publisher<Float>::SharedPtr _PubLK_Ref;
    rclcpp::Publisher<Float>::SharedPtr _PubRH_Ref;
    rclcpp::Publisher<Float>::SharedPtr _PubRK_Ref;

private: // ROS subscriptions
    rclcpp::Subscription<DevToggle>::SharedPtr _SubDevToggle;
    rclcpp::Subscription<ExternalParameter>::SharedPtr _SubExternalParameter;
    rclcpp::Subscription<FrictionParameter>::SharedPtr _SubFrictionParameter;
    rclcpp::Subscription<GaitParameter>::SharedPtr _SubGaitParameter;
    rclcpp::Subscription<HeartBeat>::SharedPtr _SubHeartBeat;
    rclcpp::Subscription<PatientParameter>::SharedPtr _SubPatientParameter;
    rclcpp::Subscription<PDParameter>::SharedPtr _SubPDParameter;
    rclcpp::Subscription<SitToStandParameter>::SharedPtr _SubSitToStandParameter;
    rclcpp::Subscription<Float>::SharedPtr _SubTorqueLimit;
    rclcpp::Subscription<TorqueParameter>::SharedPtr _SubTorqueParameter;
    rclcpp::Subscription<UserCommand>::SharedPtr _SubUserCommand;
};

#endif//__EXO_NODE_HPP_
