#include "X2DemoMachineROS.h"

X2DemoMachineROS::X2DemoMachineROS(X2Robot *robot, X2DemoState *x2DemoState, ros::NodeHandle& nodeHandle):
        robot_(robot),
        x2DemoState_(x2DemoState),
        nodeHandle_(&nodeHandle)
{

    requestedJointTorquesMsg_.data.resize(12);
    desiredJointReferencePositionsMsg_.data.resize(4);

#ifndef SIM  // if simulation, these will be published by Gazebo
    jointStatePublisher_ = nodeHandle_->advertise<sensor_msgs::JointState>("joint_states", 10);
    interactionForcePublisher_ = nodeHandle_->advertise<CORC::X2Array>("interaction_forces", 10);
    for(int i = 0; i< X2_NUM_GRF_SENSORS; i++){
        groundReactionForcePublisher_[i] = nodeHandle_->advertise<geometry_msgs::WrenchStamped>
                ("grf_" + grfFramesArray_[i], 10);
    }
#endif //todo check which ones should be published if SIM

    calibrateForceSensorsService_ = nodeHandle_->advertiseService("calibrate_force_sensors", &X2DemoMachineROS::calibrateForceSensorsCallback, this);
    emergencyStopService_ = nodeHandle_->advertiseService("emergency_stop", &X2DemoMachineROS::emergencyStopCallback, this);
    startHomingService_ = nodeHandle_->advertiseService("start_homing", &X2DemoMachineROS::startHomingCallback, this);
    imuCalibrationService_ = nodeHandle_->advertiseService("calibrate_imu", &X2DemoMachineROS::calibrateIMUCallback, this);
    interactionForceCommand_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    gainUpdateSubscriber_ = nodeHandle_->subscribe("joint_gains", 1, &X2DemoMachineROS::updateGainCallback, this);
    gainLimitUpdateSubscriber_ = nodeHandle_->subscribe("joint_gain_coeff", 1, &X2DemoMachineROS::updateGainLimitCallback, this); 
    requestedTorquePublisher_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>("joint_output", 10);
    referenceJointPositionsPublisher_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>("joint_reference", 10);
    frictionCompensationSubscriber_ = nodeHandle_->subscribe("joint_friction_compensation", 1, &X2DemoMachineROS::updateFrictionCompensationCallback, this);
    jointCommandSubscriber_ = nodeHandle_->subscribe("joint_parameters", 1, &X2DemoMachineROS::updateExternalTorquesCallback, this);
}

X2DemoMachineROS::~X2DemoMachineROS() {
    ros::shutdown();
}

void X2DemoMachineROS::initialize() {
    spdlog::debug("X2DemoMachineROS::init()");

}

void X2DemoMachineROS::update() {
#ifndef SIM  // if simulation, these will be published by Gazebo
    publishJointStates();
    publishInteractionForces();
    publishGroundReactionForces();
#endif
    publishRequestedJointTorques();
    publishJointReferencePositions();
}

void X2DemoMachineROS::publishJointStates() {
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

void X2DemoMachineROS::publishInteractionForces() {
    Eigen::VectorXd interactionForces = robot_->getInteractionForce();

    interactionForceMsg_.header.stamp = ros::Time::now();

    interactionForceMsg_.name.resize(X2_NUM_GENERALIZED_COORDINATES);
    interactionForceMsg_.data.resize(X2_NUM_GENERALIZED_COORDINATES);

    interactionForceMsg_.name[0] = "backpack_force";
    interactionForceMsg_.name[1] = "left_thigh_force";
    interactionForceMsg_.name[2] = "left_shank_force";
    interactionForceMsg_.name[3] = "right_thigh_force";
    interactionForceMsg_.name[4] = "right_shank_force";

    for(int id = 0; id < X2_NUM_GENERALIZED_COORDINATES; id++){
        interactionForceMsg_.data[id] = interactionForces[id];
    }

    interactionForcePublisher_.publish(interactionForceMsg_);
}

void X2DemoMachineROS::publishGroundReactionForces() {
    Eigen::VectorXd groundReactionForces = -robot_->getGroundReactionForces();

    for(int i = 0; i< X2_NUM_GRF_SENSORS; i++){
        groundReactionForceMsgArray_[i].header.stamp = ros::Time::now();
        groundReactionForceMsgArray_[i].header.frame_id = grfFramesArray_[i];
        groundReactionForceMsgArray_[i].wrench.force.z = groundReactionForces[i];
        groundReactionForcePublisher_[i].publish(groundReactionForceMsgArray_[i]);
    }
}

void X2DemoMachineROS::publishRequestedJointTorques() {

    Eigen::VectorXd desiredJointTorques = x2DemoState_->getDesiredJointTorques();
    Eigen::VectorXd pJointTorques = x2DemoState_->getDesiredJointTorquesPSplit();
    Eigen::VectorXd dJointTorques = x2DemoState_->getDesiredJointTorquesDSplit();

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

void X2DemoMachineROS::publishJointReferencePositions() {

    Eigen::VectorXd& desiredJointPositions = x2DemoState_->getDesiredJointPositions();

    desiredJointReferencePositionsMsg_.data[0] = desiredJointPositions[0];
    desiredJointReferencePositionsMsg_.data[1] = desiredJointPositions[1];
    desiredJointReferencePositionsMsg_.data[2] = desiredJointPositions[2];
    desiredJointReferencePositionsMsg_.data[3] = desiredJointPositions[3];

    referenceJointPositionsPublisher_.publish(desiredJointReferencePositionsMsg_);
}

void X2DemoMachineROS::setNodeHandle(ros::NodeHandle &nodeHandle) {
    nodeHandle_ = &nodeHandle;
}

ros::NodeHandle & X2DemoMachineROS::getNodeHandle() {

    return *nodeHandle_;
}

bool X2DemoMachineROS::calibrateForceSensorsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    bool success = true;
    for(int id = 0; id < X2_NUM_FORCE_SENSORS + X2_NUM_GRF_SENSORS; id++){
        success = success & robot_->forceSensors[id]->sendInternalCalibrateSDOMessage();
    }

    success = success & robot_->calibrateForceSensors();
    res.success = success;

    return success;
}

bool X2DemoMachineROS::startHomingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    std::vector<int> homingDirection{1, 1, 1, 1};
    res.success = robot_->homing(homingDirection);
    return true;
}

bool X2DemoMachineROS::emergencyStopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    std::raise(SIGTERM); //Clean exit
    return true;
}

bool X2DemoMachineROS::calibrateIMUCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    res.success = robot_->setBackpackIMUMode(IMUOutputMode::QUATERNION_GYRO);

//    robot_->calibrateContactIMUAngles(2.0);
    return true;
}

void X2DemoMachineROS::updateGainCallback(const std_msgs::Float64MultiArray::ConstPtr& gains) {
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

    x2DemoState_->jointControllers[0](left_kp_gains, left_kd_gains);
    x2DemoState_->jointControllers[1](right_kp_gains, right_kd_gains);
}

void X2DemoMachineROS::updateGainLimitCallback(const std_msgs::Float64MultiArray::ConstPtr& alphas) {
    double hip_alpha1 = alphas->data[0];
    double hip_alpha2 = alphas->data[1];
    double knee_alpha1 = alphas->data[2];
    double knee_alpha2 = alphas->data[3];

    x2DemoState_->jointControllers[0].set_alpha({hip_alpha1, knee_alpha1}, {hip_alpha2, knee_alpha2});
    x2DemoState_->jointControllers[1].set_alpha({hip_alpha1, knee_alpha1}, {hip_alpha2, knee_alpha2});
}

void X2DemoMachineROS::updateExternalTorquesCallback(const std_msgs::Float64MultiArray::ConstPtr& externalTorques) {

    auto joint0_debug_torque = externalTorques->data[0];
    auto joint1_debug_torque = externalTorques->data[1];
    auto joint2_debug_torque = externalTorques->data[2];
    auto joint3_debug_torque = externalTorques->data[3];
    auto torqueLimit = externalTorques->data[4];
    auto refPos1 = externalTorques->data[5];
    auto refPos2 = externalTorques->data[6];
    auto refPosPeriod = floor(externalTorques->data[7]);
    auto rateLimit = externalTorques->data[8];

    x2DemoState_->debugTorques[0] = joint0_debug_torque;
    x2DemoState_->debugTorques[1] = joint1_debug_torque;
    x2DemoState_->debugTorques[2] = joint2_debug_torque;
    x2DemoState_->debugTorques[3] = joint3_debug_torque;
    x2DemoState_->maxTorqueLimit = torqueLimit;
    x2DemoState_->refPos1 = refPos1;
    x2DemoState_->refPos2 = refPos2;
    x2DemoState_->refPosPeriod = refPosPeriod;
    x2DemoState_->rateLimit = rateLimit;
}

void X2DemoMachineROS::updateFrictionCompensationCallback(const std_msgs::Float64MultiArray::ConstPtr& frictionTorques) {
    x2DemoState_->frictionCompensationTorques[0] = frictionTorques->data[0];
    x2DemoState_->frictionCompensationTorques[1] = frictionTorques->data[1];
    x2DemoState_->frictionCompensationTorques[2] = frictionTorques->data[2];
    x2DemoState_->frictionCompensationTorques[3] = frictionTorques->data[3];
    x2DemoState_->frictionCompensationTorques[4] = frictionTorques->data[4];
    x2DemoState_->frictionCompensationTorques[5] = frictionTorques->data[5];
    x2DemoState_->frictionCompensationTorques[6] = frictionTorques->data[6];
    x2DemoState_->frictionCompensationTorques[7] = frictionTorques->data[7];
}