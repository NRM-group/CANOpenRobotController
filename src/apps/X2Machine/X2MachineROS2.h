#ifndef SRC_X2MACHINEROS2_H
#define SRC_X2MACHINEROS2_h

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "exo_msgs/msg/pd_parameter.hpp"
#include "exo_msgs/msg/external_parameter.hpp"
#include "exo_msgs/msg/friction_parameter.hpp"
#include "exo_msgs/msg/dev_toggle.hpp"
// #include "exo_msgs/msg/corc.hpp" //Deprecated?
// #include "exo_msgs/msg/output.hpp" //Deprecated?



#include "x2_ik_ros.hpp"
#include "X2Robot.h"
#include "states/X2FollowerState.h"

#include <array>

using std::placeholders::_1;

class X2MachineROS2 {

using Tv = ctrl::BaseController<double, X2_NUM_JOINTS>::_Tv_;
using Tm = ctrl::BaseController<double, X2_NUM_JOINTS>::_Tm_;

public:
    X2MachineROS2(X2Robot* robot, X2FollowerState* x2State, std::shared_ptr<rclcpp::Node>& node);
    ~X2MachineROS2();

    void initalise();
    void update(void);
    void publishJointStates(void);
    void publishRequestedJointTorques(void);
    void publishJointReferencePositions(void);
    void publishControllerOutputs(void);

private:
    X2Robot* robot_;
    X2FollowerState* x2FollowerState_;

    LegKinematics<double> kinHandler; //Ensure that the params file that sets the length is the same for x2_ik_ros.h

    std::shared_ptr<rclcpp::Node> node_;

    //Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr requestedTorquePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr referenceJointPositionsPublisher_;
    // rclcpp::Publisher<exo_msgs::msg::Output>::SharedPtr controllerOutputPublisher_; //Deprecated?
    rclcpp::Publisher<exo_msgs::msg::Endpoint>::SharedPtr ankleEndpointPublisher_;

    //Subscribers    
    // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gainUpdateSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
    rclcpp::Subscription<exo_msgs::msg::PDParameter>::SharedPtr gainUpdateSubscriber_;
    rclcpp::Subscription<exo_msgs::msg::ExternalParameter>::SharedPtr externalUpdateSubscriber_;
    rclcpp::Subscription<exo_msgs::msg::FrictionParameter>::SharedPtr frictionUpdateSubscriber_;
    rclcpp::Subscription<exo_msgs::msg::DevToggle>::SharedPtr enableUpdateSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torqueLimitSubscriber_;

    // rclcpp::Subscription<exo_msgs::msg::Corc>::SharedPtr corcParamsSubscriber_; //Deprecated
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr frictionCompensationSubscriber_;
    // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr jointCommandSubscriber_;

    sensor_msgs::msg::JointState jointStateMsg_;
    exo_msgs::msg::Endpoint ankleEndpoitsMsg_;
    std_msgs::msg::Float64MultiArray requestedJointTorquesMsg_;
    std_msgs::msg::Float64MultiArray desiredJointReferencePositionsMsg_;
    // exo_msgs::msg::Output controllerOutputsMsg_;
    
    // void updateExternalTorquesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr externalTorques);
    void jointRefCallback(const sensor_msgs::msg::JointState::SharedPtr jointRef);
    void updateGainCallback(const exo_msgs::msg::PDParameter::SharedPtr gains);
    void externalForceCallback(const exo_msgs::msg::ExternalParameter::SharedPtr ext);
    void frictionForceCallback(const exo_msgs::msg::FrictionParameter::SharedPtr fric);
    void enablerCallback(const exo_msgs::msg::DevToggle::SharedPtr enable);
    void torqueLimitCallback(const std_msgs::msg::Float64::SharedPtr limit);
    // void corcParamCallback(const exo_msgs::msg::Corc::SharedPtr corcParams);
};

#endif