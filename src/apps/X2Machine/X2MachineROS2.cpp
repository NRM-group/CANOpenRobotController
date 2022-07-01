#include "X2MachineROS2.h"

X2MachineROS2::X2MachineROS2(X2Robot* robot, X2FollowerState* x2FollowerState, std::shared_ptr<rclcpp::Node>& node) :
        robot_(robot),
        x2FollowerState_(x2FollowerState),
        node_(node)
{
    requestedJointTorquesMsg_.data.resize(3 * X2_NUM_JOINTS);
    desiredJointReferencePositionsMsg_.data.resize(X2_NUM_JOINTS);
    jointStateMsg_.name.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.position.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.velocity.resize(X2_NUM_JOINTS + 1);
    jointStateMsg_.effort.resize(X2_NUM_JOINTS + 1);

#ifndef SIM  // if simulation, these will be published by Gazebo
    jointStatePublisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
#endif

    ankleEndpointPublisher_ = node_->create_publisher<x2_msgs::msg::Endpoint>("ankle_endpoints", 10);
    requestedTorquePublisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("joint_output", 10);
    referenceJointPositionsPublisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("actual_joint_references", 10);

    controllerOutputPublisher_ = node_->create_publisher<x2_msgs::msg::Output>("controller_outputs",10);

    gainUpdateSubscriber_ = node_->create_subscription<x2_msgs::msg::PD>("pd_params", 1, std::bind(&X2MachineROS2::updateGainCallback, this, _1));
    // frictionCompensationSubscriber_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("joint_friction_compensation", 1, std::bind(&X2MachineROS2::updateFrictionCompensationCallback, this, _1));
    // jointCommandSubscriber_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("joint_parameters", 1, std::bind(&X2MachineROS2::updateExternalTorquesCallback, this, _1));
    jointStateSubscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>("joint_references", 1, std::bind(&X2MachineROS2::jointRefCallback, this, _1));
    corcParamsSubscriber_ = node_->create_subscription<x2_msgs::msg::Corc>("corc_params",1, std::bind(&X2MachineROS2::corcParamCallback, this, _1));

    externalUpdateSubscriber_ = node_->create_subscription<x2_msgs::msg::External>("external_params",1, std::bind(&X2MachineROS2::externalForceCallback, this, _1));
    frictionUpdateSubscriber_ = node_->create_subscription<x2_msgs::msg::Friction>("friction_params", 1, std::bind(&X2MachineROS2::frictionForceCallback, this, _1));
    enableUpdateSubscriber_ = node_->create_subscription<x2_msgs::msg::Enable>("enable",1, std::bind(&X2MachineROS2::enablerCallback, this, _1));

}

X2MachineROS2::~X2MachineROS2() {
    rclcpp::shutdown();
}

void X2MachineROS2::initalise() {
    rclcpp::spin(std::make_shared<X2IKROS>());
    spdlog::info("X2 ROS state machine initalised");
}

void X2MachineROS2::update() {
#ifndef SIM  // if simulation, these will be published by Gazebo
    publishJointStates();
#endif
    publishRequestedJointTorques();
    publishJointReferencePositions();
    publishControullerOutputs();
}

void X2MachineROS2::publishControullerOutputs(void){ 
    //TODO Gravity
    Eigen::VectorXd  PDout = x2FollowerState_->PDCntrl->output();
    Eigen::VectorXd  ExtOut = x2FollowerState_->ExtCntrl->output();
    Eigen::VectorXd  FricOut = x2FollowerState_->FricCntrl->output();

    controllerOutputsMsg_.pd[0] = PDout[0];
    controllerOutputsMsg_.pd[1] = PDout[1];
    controllerOutputsMsg_.pd[2] = PDout[2];
    controllerOutputsMsg_.pd[3] = PDout[3];

    controllerOutputsMsg_.external[0] = ExtOut[0];
    controllerOutputsMsg_.external[1] = ExtOut[1];
    controllerOutputsMsg_.external[2] = ExtOut[2];
    controllerOutputsMsg_.external[3] = ExtOut[3];

    controllerOutputsMsg_.friction[0] = FricOut[0];
    controllerOutputsMsg_.friction[1] = FricOut[1];
    controllerOutputsMsg_.friction[2] = FricOut[2];
    controllerOutputsMsg_.friction[3] = FricOut[3];
    
    controllerOutputsMsg_.total[0] = x2FollowerState_->getDesiredJointTorques()[0];
    controllerOutputsMsg_.total[1] = x2FollowerState_->getDesiredJointTorques()[1];
    controllerOutputsMsg_.total[2] = x2FollowerState_->getDesiredJointTorques()[2];
    controllerOutputsMsg_.total[3] = x2FollowerState_->getDesiredJointTorques()[3];

    controllerOutputPublisher_->publish(controllerOutputsMsg_);
}


void X2MachineROS2::publishJointStates(void) {
    Eigen::VectorXd jointPositions = robot_->getPosition();
    Eigen::VectorXd jointVelocities = robot_->getVelocity();
    Eigen::VectorXd jointTorques = robot_->getTorque();
    Eigen::VectorXd ankleEndpoints = kinHandler.fow_kin(robot_->getPosition());
    jointStateMsg_.header.stamp = node_->now();
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
    ankleEndpoitsMsg_.left_x = ankleEndpoints(0);
    ankleEndpoitsMsg_.left_y = ankleEndpoints(1);
    ankleEndpoitsMsg_.right_x = ankleEndpoints(2);
    ankleEndpoitsMsg_.right_y = ankleEndpoints(3);
    
    ankleEndpointPublisher_->publish(ankleEndpoitsMsg_);
    jointStateMsg_.position[4] = robot_->getBackPackAngleOnMedianPlane() - M_PI_2;
    jointStatePublisher_->publish(jointStateMsg_);
}

void X2MachineROS2::publishRequestedJointTorques(void) {

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

    requestedTorquePublisher_->publish(requestedJointTorquesMsg_);
}

void X2MachineROS2::publishJointReferencePositions(void) {

    Eigen::VectorXd& desiredJointPositions = x2FollowerState_->getDesiredJointPositions();

    desiredJointReferencePositionsMsg_.data[0] = desiredJointPositions[0];
    desiredJointReferencePositionsMsg_.data[1] = desiredJointPositions[1];
    desiredJointReferencePositionsMsg_.data[2] = desiredJointPositions[2];
    desiredJointReferencePositionsMsg_.data[3] = desiredJointPositions[3];

    referenceJointPositionsPublisher_->publish(desiredJointReferencePositionsMsg_);
}

void X2MachineROS2::updateGainCallback(const x2_msgs::msg::PD::SharedPtr gains) {
    Tm kp, kd;
    Tv alphaMax(gains->alpha_max.data()), alphaMin(gains->alpha_min.data());
    kp << gains->left_kp[0], gains->left_kp[1], 0 , 0,
                gains->left_kp[2], gains->left_kp[3], 0, 0,
                0, 0, gains->right_kp[0], gains->right_kp[1],
                0, 0, gains->right_kp[2], gains->right_kp[3];
    
    kd << gains->left_kd[0], gains->left_kd[1], 0 , 0,
                gains->left_kd[2], gains->left_kd[3], 0, 0,
                0, 0, gains->right_kp[0], gains->right_kp[1],
                0, 0, gains->right_kp[2], gains->right_kp[3];
    x2FollowerState_->PDCntrl->set_gains(kp,kd);
    x2FollowerState_->PDCntrl->set_alphas(alphaMin, alphaMax);
}

void X2MachineROS2::jointRefCallback(const sensor_msgs::msg::JointState::SharedPtr angles) {
    //Update the joint references in X2follower state
    Tv jointRef(angles->position.data());
    x2FollowerState_->desiredJointReferences_ = jointRef;
}

void X2MachineROS2::externalForceCallback(const x2_msgs::msg::External::SharedPtr ext){
    Tv torque(ext->torque.data());
    x2FollowerState_->ExtCntrl->set_external_torque(torque);
}

void X2MachineROS2::frictionForceCallback(const x2_msgs::msg::Friction::SharedPtr fric) {
    Tv pos_c(fric->positive_c.data()), neg_c(fric->negative_c.data()); 
    Tv pos_m(fric->positive_m.data()), neg_m(fric->positive_m.data());
    x2FollowerState_->FricCntrl->set_vel_c(pos_c, neg_c);
    x2FollowerState_->FricCntrl->set_vel_m(pos_m, neg_m);

}

void X2MachineROS2::enablerCallback(const x2_msgs::msg::Enable::SharedPtr enable) {
    //TODO
}


void X2MachineROS2::corcParamCallback(const x2_msgs::msg::Corc::SharedPtr corcParams) {
    x2FollowerState_->rateLimit = corcParams->reference_limit;
    x2FollowerState_->maxTorqueLimit = corcParams->maximum_torque;
    x2FollowerState_->posReader.updateTrajectoryTime(corcParams->trajectory_period);
}
