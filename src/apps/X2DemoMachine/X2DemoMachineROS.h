/**
 * /file X2DemoMachineROS.h
 * /author Emek Baris Kucuktabak
 * /brief ROS part of the X2DemoMachine
 * /version 0.1
 * /date 2020-07-06
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef SRC_X2DEMOMACHINEROS_H
#define SRC_X2DEMOMACHINEROS_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Trigger.h>
#include <CORC/X2Array.h>
#include <CORC/X2Acceleration.h>
#include <CORC/X2AccelerationMerge.h>

#include "X2Robot.h"
#include "X2DemoState.h"
#include "ros/ros.h"  // This state machine requires ROS

class X2DemoMachineROS {
public:
    X2DemoMachineROS(X2Robot *robot, X2DemoState *x2DemoState, ros::NodeHandle& nodeHandle);
    ~X2DemoMachineROS();

    void update(void);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void publishGroundReactionForces(void);
    void publishRequestedJointTorques(void);
    void publishRequestedJointTorquesSeperate(void);
    void publishJointReferencePositions(void);
    void initialize();
    void setNodeHandle(ros::NodeHandle& nodeHandle);
    ros::NodeHandle& getNodeHandle();

    Eigen::VectorXd interactionForceCommand_;

private:
    ros::Publisher jointStatePublisher_;
    ros::Publisher interactionForcePublisher_;
    ros::Publisher groundReactionForcePublisher_[X2_NUM_GRF_SENSORS];
    ros::Publisher requestedTorquePublisher_;
    ros::Publisher requestedJoint1TorquePublisher_;
    ros::Publisher requestedJoint2TorquePublisher_;
    ros::Publisher requestedJoint3TorquePublisher_;
    ros::Publisher requestedJoint4TorquePublisher_;
    ros::Publisher referenceJoint1PositionsPublisher_;
    ros::Publisher referenceJoint2PositionsPublisher_;
    ros::Publisher referenceJoint3PositionsPublisher_;
    ros::Publisher referenceJoint4PositionsPublisher_;

    ros::Subscriber gainUpdateSubscriber_;
    ros::Subscriber gainLimitUpdateSubscriber_;
    ros::Subscriber maxTorqueSubscriber_;
    ros::Subscriber frictionCompensationSubscriber_;  
    ros::Subscriber jointCommandSubscriber_;

    ros::ServiceServer calibrateForceSensorsService_;
    ros::ServiceServer startHomingService_;
    ros::ServiceServer emergencyStopService_;
    ros::ServiceServer imuCalibrationService_;

    sensor_msgs::JointState jointStateMsg_;
    std_msgs::Float64MultiArray requestedJointTorquesMsg_;
    std_msgs::Float64 jointReferencePositionsMsg_;
    std_msgs::Float64 requestedJointTorqueSeperateMsg_;
    CORC::X2Array interactionForceMsg_;
    geometry_msgs::WrenchStamped groundReactionForceMsgArray_[X2_NUM_GRF_SENSORS];

    std::string grfFramesArray_[X2_NUM_GRF_SENSORS] = {"left_lower_shank", "right_lower_shank"};

    X2Robot *robot_;
    X2DemoState *x2DemoState_;

    bool calibrateForceSensorsCallback(std_srvs::Trigger::Request& req,
                                       std_srvs::Trigger::Response& res);

    bool startHomingCallback(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res);

    bool emergencyStopCallback(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);

    bool calibrateIMUCallback(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res);
    
    void updateFrictionCompensationCallback(const std_msgs::Float64MultiArray::ConstPtr& frictionTorques);
    void updateExternalTorquesCallback(const std_msgs::Float64MultiArray::ConstPtr& externalTorques);

    ros::NodeHandle* nodeHandle_;
};

#endif  //SRC_X2DEMOMACHINEROS_H
