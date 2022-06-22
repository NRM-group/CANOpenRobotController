#ifndef SRC_X2MACHINEROS2_H
#define SRC_X2MACHINEROS2_h

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rclcpp/rclcpp.hpp"

#include "X2Robot.h"
#include "states/X2FollowerState.h"

using std::placeholders::_1;

class X2MachineROS2 {

public:
    X2MachineROS2(X2Robot* robot, X2FollowerState* x2State, std::shared_ptr<rclcpp::Node>& node);
    ~X2MachineROS2();

    void initalise();
    void update(void);
    void publishJointStates(void);
    void publishRequestedJointTorques(void);
    void publishJointReferencePositions(void);

private:
    X2Robot* robot_;
    X2FollowerState* x2FollowerState_;

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr requestedTorquePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr referenceJointPositionsPublisher_;
    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gainUpdateSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gainLimitUpdateSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr frictionCompensationSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr jointCommandSubscriber_;

    sensor_msgs::msg::JointState jointStateMsg_;
    std_msgs::msg::Float64MultiArray requestedJointTorquesMsg_;
    std_msgs::msg::Float64MultiArray desiredJointReferencePositionsMsg_;

    void updateGainCallback(const std_msgs::msg::Float64MultiArray::SharedPtr gains);
    void updateGainLimitCallback(const std_msgs::msg::Float64MultiArray::SharedPtr alphas);
    void updateFrictionCompensationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr frictionTorques);
    void updateExternalTorquesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr externalTorques);
};

#endif