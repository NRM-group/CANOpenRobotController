#include "X2MachineROS.h"

X2MachineROS::X2MachineROS(X2Robot* robot, X2FollowerState* x2FollowerState, ros::NodeHandle& nodeHandle) :
        robot_(robot),
        x2FollowerState_(x2FollowerState),
        nodeHandle_(&nodeHandle)
{
    requestedJointTorquesMsg_.data.resize(3 * X2_NUM_JOINTS);
    desiredJointReferencePositionsMsg_.data.resize(X2_NUM_JOINTS);

#ifndef SIM // if simulation, these will be published by Gazebo
    jointStatePublisher_ = nodeHandle_->advertise<sensor_msgs::JointState>("joint_states", 10);
#endif

    gainUpdateSubscriber_ = nodeHandle_->subscribe("joint_gains", 1, &X2MachineROS::updateGainCallback, this);
    gainLimitUpdateSubscriber_ = nodeHandle_->subscribe("joint_gain_coeff", 1, &X2MachineROS::updateGainLimitCallback, this); 
    requestedTorquePublisher_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>("joint_output", 10);
    referenceJointPositionsPublisher_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>("joint_reference", 10);
    frictionCompensationSubscriber_ = nodeHandle_->subscribe("joint_friction_compensation", 1, &X2MachineROS::updateFrictionCompensationCallback, this);
    jointCommandSubscriber_ = nodeHandle_->subscribe("joint_parameters", 1, &X2MachineROS::updateExternalTorquesCallback, this);
}

X2MachineROS::~X2MachineROS() {
    ros::shutdown();
}

void X2MachineROS::initalise() {
    spdlog::info("X2 ROS state machine initalised");
}

void X2MachineROS::update() {
#ifndef SIM  // if simulation, these will be published by Gazebo
    publishJointStates();
#endif
    publishRequestedJointTorques();
    publishJointReferencePositions();
}

void X2MachineROS::publishJointStates() {
    Eigen::VectorXd jointPositions = robot_->getPosition();
    Eigen::VectorXd jointVelocities = robot_->getVelocity();
    Eigen::VectorXd jointTorques = robot_->getTorque();

    jointStateMsg_.header.stamp = ros::Time::now();
    jointStateMsg_.name.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.position.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.velocity.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.effort.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.name[0] = "left_hip_joint";
    jointStateMsg_.name[1] = "left_knee_joint";
    jointStateMsg_.name[2] = "right_hip_joint";
    jointStateMsg_.name[3] = "right_knee_joint";
    jointStateMsg_.name[4] = "world_to_backpack";

    for(int id = 0; id < X2_NUM_JOINTS; id++){
        jointStateMsg_.position[id] = jointPositions[id];
        jointStateMsg_.velocity[id] = jointVelocities[id];
        jointStateMsg_.effort[id] = jointTorques[id];
    }

    jointStateMsg_.position[4] = robot_->getBackPackAngleOnMedianPlane() - M_PI_2;
    jointStatePublisher_.publish(jointStateMsg_);
}

void X2MachineROS::publishRequestedJointTorques() {

    Eigen::VectorXd desiredJointTorques = x2FollowerState_->getDesiredJointTorques();
    Eigen::VectorXd pJointTorques = x2FollowerState_->getDesiredJointTorquesPSplit();
    Eigen::VectorXd dJointTorques = x2FollowerState_->getDesiredJointTorquesDSplit();

    requestedJointTorquesMsg_.data[0] = desiredJointTorques[0];
    requestedJointTorquesMsg_.data[1] = pJointTorques[0];
    requestedJointTorquesMsg_.data[2] = dJointTorques[0];

    requestedJointTorquesMsg_.data[3] = desiredJointTorques[1];
    requestedJointTorquesMsg_.data[4] = pJointTorques[1];
    requestedJointTorquesMsg_.data[5] = dJointTorques[1];

    requestedJointTorquesMsg_.data[6] = desiredJointTorques[2];
    requestedJointTorquesMsg_.data[7] = pJointTorques[2];
    requestedJointTorquesMsg_.data[8] = dJointTorques[2];

    requestedJointTorquesMsg_.data[9] = desiredJointTorques[3];
    requestedJointTorquesMsg_.data[10] = pJointTorques[3];
    requestedJointTorquesMsg_.data[11] = dJointTorques[3];

    requestedTorquePublisher_.publish(requestedJointTorquesMsg_);
}

void X2MachineROS::publishJointReferencePositions() {

    Eigen::VectorXd& desiredJointPositions = x2FollowerState_->getDesiredJointPositions();

    desiredJointReferencePositionsMsg_.data[0] = desiredJointPositions[0];
    desiredJointReferencePositionsMsg_.data[1] = desiredJointPositions[1];
    desiredJointReferencePositionsMsg_.data[2] = desiredJointPositions[2];
    desiredJointReferencePositionsMsg_.data[3] = desiredJointPositions[3];

    referenceJointPositionsPublisher_.publish(desiredJointReferencePositionsMsg_);
}

void X2MachineROS::updateGainCallback(const std_msgs::Float64MultiArray::ConstPtr& gains) {
    double left_hip_kp = gains->data[0];
    double left_hip_kd = gains->data[1];
    double left_knee_kp = gains->data[2];
    double left_knee_kd = gains->data[3];
    double right_hip_kp = gains->data[4];
    double right_hip_kd = gains->data[5];
    double right_knee_kp = gains->data[6]; 
    double right_knee_kd = gains->data[7];

    Eigen::Matrix2d left_kp_gains;
    Eigen::Matrix2d left_kd_gains;
    Eigen::Matrix2d right_kp_gains;
    Eigen::Matrix2d right_kd_gains;

    left_kp_gains << left_hip_kp, 0,
                     0, left_knee_kp;
    left_kd_gains << left_hip_kd, 0,
                     0, left_knee_kd;
    right_kp_gains << right_hip_kp, 0,
                      0, right_knee_kp;
    right_kd_gains << right_hip_kd, 0,
                      0, right_knee_kd;

    x2FollowerState_->jointControllers[0](left_kp_gains, left_kd_gains);
    x2FollowerState_->jointControllers[1](right_kp_gains, right_kd_gains);
}

void X2MachineROS::updateGainLimitCallback(const std_msgs::Float64MultiArray::ConstPtr& alphas) {
    double hip_alpha1 = alphas->data[0];
    double hip_alpha2 = alphas->data[1];
    double knee_alpha1 = alphas->data[2];
    double knee_alpha2 = alphas->data[3];

    x2FollowerState_->jointControllers[0].set_alpha({hip_alpha1, knee_alpha1}, {hip_alpha2, knee_alpha2});
    x2FollowerState_->jointControllers[1].set_alpha({hip_alpha1, knee_alpha1}, {hip_alpha2, knee_alpha2});
}

void X2MachineROS::updateExternalTorquesCallback(const std_msgs::Float64MultiArray::ConstPtr& externalTorques) {

    auto joint0_debug_torque = externalTorques->data[0];
    auto joint1_debug_torque = externalTorques->data[1];
    auto joint2_debug_torque = externalTorques->data[2];
    auto joint3_debug_torque = externalTorques->data[3];
    auto torqueLimit = externalTorques->data[4];
    auto refPos1 = externalTorques->data[5];
    auto refPos2 = externalTorques->data[6];
    auto refPosPeriod = floor(externalTorques->data[7]);
    auto rateLimit = externalTorques->data[8];

    x2FollowerState_->debugTorques[0] = joint0_debug_torque;
    x2FollowerState_->debugTorques[1] = joint1_debug_torque;
    x2FollowerState_->debugTorques[2] = joint2_debug_torque;
    x2FollowerState_->debugTorques[3] = joint3_debug_torque;
    x2FollowerState_->maxTorqueLimit = torqueLimit;
    x2FollowerState_->refPos1 = refPos1;
    x2FollowerState_->refPos2 = refPos2;
    x2FollowerState_->refPosPeriod = refPosPeriod;
    x2FollowerState_->rateLimit = rateLimit;
}

void X2MachineROS::updateFrictionCompensationCallback(const std_msgs::Float64MultiArray::ConstPtr& frictionTorques) {
    x2FollowerState_->frictionCompensationTorques[0] = frictionTorques->data[0];
    x2FollowerState_->frictionCompensationTorques[1] = frictionTorques->data[1];
    x2FollowerState_->frictionCompensationTorques[2] = frictionTorques->data[2];
    x2FollowerState_->frictionCompensationTorques[3] = frictionTorques->data[3];
    x2FollowerState_->frictionCompensationTorques[4] = frictionTorques->data[4];
    x2FollowerState_->frictionCompensationTorques[5] = frictionTorques->data[5];
    x2FollowerState_->frictionCompensationTorques[6] = frictionTorques->data[6];
    x2FollowerState_->frictionCompensationTorques[7] = frictionTorques->data[7];
}