#ifndef SRC_X2MACHINEROS_H
#define SRC_X2MACHINEROS_h

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include "ros/ros.h"
#include "X2Robot.h"
#include "states/X2FollowerState.h"

class X2MachineROS {

public:
    X2MachineROS(X2Robot* robot, X2FollowerState* x2State, ros::NodeHandle& nodeHandle);
    ~X2MachineROS();

    void initalise();
    void update(void);
    void publishJointStates(void);
    void publishRequestedJointTorques(void);
    void publishJointReferencePositions(void);

private:
    X2Robot* robot_;
    X2FollowerState* x2FollowerState_;

    ros::NodeHandle* nodeHandle_;

    ros::Publisher jointStatePublisher_;
    ros::Publisher requestedTorquePublisher_;
    ros::Publisher referenceJointPositionsPublisher_;

    ros::Subscriber gainUpdateSubscriber_;
    ros::Subscriber gainLimitUpdateSubscriber_;
    ros::Subscriber maxTorqueSubscriber_;
    ros::Subscriber frictionCompensationSubscriber_;  
    ros::Subscriber jointCommandSubscriber_;

    sensor_msgs::JointState jointStateMsg_;
    std_msgs::Float64MultiArray requestedJointTorquesMsg_;
    std_msgs::Float64MultiArray desiredJointReferencePositionsMsg_;

    void updateGainCallback(const std_msgs::Float64MultiArray::ConstPtr& gains);
    void updateGainLimitCallback(const std_msgs::Float64MultiArray::ConstPtr& alphas);
    void updateFrictionCompensationCallback(const std_msgs::Float64MultiArray::ConstPtr& frictionTorques);
    void updateExternalTorquesCallback(const std_msgs::Float64MultiArray::ConstPtr& externalTorques);
};

#endif