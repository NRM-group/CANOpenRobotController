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

#ifndef SIM
    jointStatePublisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
#endif

    ankleEndpointPublisher_ = node_->create_publisher<exo_msgs::msg::Endpoint>("ankle_endpoints", 10);
    requestedTorquePublisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("joint_output", 10);
    referenceJointPositionsPublisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("actual_joint_references", 10);

    // controllerOutputPublisher_ = node_->create_publisher<exo_msgs::msg::Output>("controller_outputs",10);
    torqueLimitSubscriber_ = node_->create_subscription<std_msgs::msg::Float64>("maximum_torque", 1, std::bind(&X2MachineROS2::torqueLimitCallback, this, _1));
    gainUpdateSubscriber_ = node_->create_subscription<exo_msgs::msg::PDParameter>("pd_parameters", 1, std::bind(&X2MachineROS2::updateGainCallback, this, _1));
    jointStateSubscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>("joint_references", 1, std::bind(&X2MachineROS2::jointRefCallback, this, _1));
    // corcParamsSubscriber_ = node_->create_subscription<exo_msgs::msg::Corc>("corc_params",1, std::bind(&X2MachineROS2::corcParamCallback, this, _1));

    externalUpdateSubscriber_ = node_->create_subscription<exo_msgs::msg::ExternalParameter>("external_parameters",1, std::bind(&X2MachineROS2::externalForceCallback, this, _1));
    frictionUpdateSubscriber_ = node_->create_subscription<exo_msgs::msg::FrictionParameter>("friction_parameters", 1, std::bind(&X2MachineROS2::frictionForceCallback, this, _1));
    enableUpdateSubscriber_ = node_->create_subscription<exo_msgs::msg::DevToggle>("enable",1, std::bind(&X2MachineROS2::enablerCallback, this, _1));

}

X2MachineROS2::~X2MachineROS2() {
    rclcpp::shutdown();
}

void X2MachineROS2::initalise() {
    rclcpp::spin(std::make_shared<X2IKROS>());
    spdlog::info("X2 ROS state machine initalised");
}

void X2MachineROS2::update() {

#ifndef SIM
    publishJointStates();
#endif
    // publishControllerOutputs();
    publishRequestedJointTorques();
    publishJointReferencePositions();
}

// void X2MachineROS2::publishControllerOutputs(void){ 
//     //TODO Gravity
//     memcpy(controllerOutputsMsg_.friction.data(), x2FollowerState_->FricCntrl->output().data(), sizeof(double) * X2_NUM_JOINTS);
//     memcpy(controllerOutputsMsg_.total.data(), x2FollowerState_->getDesiredJointTorques().data(), sizeof(double) * X2_NUM_JOINTS);
//     controllerOutputPublisher_->publish(controllerOutputsMsg_);
// }


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
    requestedJointTorquesMsg_.data[1] = x2FollowerState_->PDCntrl->get_p()[0];
    requestedJointTorquesMsg_.data[2] = x2FollowerState_->PDCntrl->get_d()[0];

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

void X2MachineROS2::updateGainCallback(const exo_msgs::msg::PDParameter::SharedPtr gains) {
    Tm kp, kd;
    Tv alphaMax(gains->alpha_max.data() + 1), alphaMin(gains->alpha_min.data() + 1);
    kp << gains->left_kp[1], gains->left_kp[2], 0 , 0,
                gains->left_kp[3], gains->left_kp[4], 0, 0,
                0, 0, gains->right_kp[1], gains->right_kp[2],
                0, 0, gains->right_kp[3], gains->right_kp[4];
    
    kd << gains->left_kd[1], gains->left_kd[2], 0 , 0,
                gains->left_kd[3], gains->left_kd[4], 0, 0,
                0, 0, gains->right_kd[1], gains->right_kd[2],
                0, 0, gains->right_kd[3], gains->right_kd[4];
    x2FollowerState_->PDCntrl->set_gains(kp,kd);
    x2FollowerState_->PDCntrl->set_alphas(alphaMin, alphaMax);
}

void X2MachineROS2::jointRefCallback(const sensor_msgs::msg::JointState::SharedPtr angles) {
    //Update the joint references in X2follower state
    Tv jointRef(angles->position.data());
    x2FollowerState_->desiredJointReferences_ = jointRef;
}

void X2MachineROS2::externalForceCallback(const exo_msgs::msg::ExternalParameter::SharedPtr ext){
    Tv torque(ext->torque.data());
    x2FollowerState_->ExtCntrl->set_external_torque(torque);
}

void X2MachineROS2::frictionForceCallback(const exo_msgs::msg::FrictionParameter::SharedPtr fric) {
    Tv b_static, b_viscous;
    memcpy(b_static.data(), fric->static_coefficient.data(), sizeof(double) * X2_NUM_JOINTS);
    memcpy(b_viscous.data(), fric->viscous_coefficient.data(), sizeof(double) * X2_NUM_JOINTS);
    x2FollowerState_->FricCntrl->set_static(b_static);
    x2FollowerState_->FricCntrl->set_viscous(b_viscous);
}

void X2MachineROS2::enablerCallback(const exo_msgs::msg::DevToggle::SharedPtr enable) {
    //TODO
}

void X2MachineROS2::torqueLimitCallback(const std_msgs::msg::Float64::SharedPtr limit) {
    x2FollowerState_->maxTorqueLimit = limit->data;
}

// void X2MachineROS2::corcParamCallback(const exo_msgs::msg::Corc::SharedPtr corcParams) {
//     x2FollowerState_->rateLimit = corcParams->reference_limit;
//     x2FollowerState_->maxTorqueLimit = corcParams->maximum_torque;
//     x2FollowerState_->posReader.updateTrajectoryTime(corcParams->trajectory_period);
// }
