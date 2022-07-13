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


    controllerOutputPublisher_ = node_->create_publisher<x2_msgs::msg::Output>("controller_outputs",10);

    strainGaugePublisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("strain_gauge", 10);
    filteredGaugePublisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("filtered_gauge", 10);
    corcParamsSubscriber_ = node_->create_subscription<x2_msgs::msg::Corc>("corc_params",1, std::bind(&X2MachineROS2::corcParamCallback, this, _1));

    frictionUpdateSubscriber_ = node_->create_subscription<x2_msgs::msg::Friction>("friction_params", 1, std::bind(&X2MachineROS2::frictionForceCallback, this, _1));

    std::vector<double> c0, c1;
    node_->declare_parameter("c0");
    node_->declare_parameter("c1");
    node_->get_parameter("c0", c0);
    node_->get_parameter("c1", c1);
    Eigen::Vector4d b_static(c0.data()), b_viscous(c1.data());
    //x2FollowerState_->friction.set_static(b_static);
    //x2FollowerState_->friction.set_viscous(b_viscous);

    std::vector<double> m, s;
    std::vector<double> l;
    node_->declare_parameter("m");
    node_->declare_parameter("l");
    node_->declare_parameter("s");
    node_->get_parameter("m", m);
    node_->get_parameter("l", l);
    node_->get_parameter("s", s);
    Eigen::Vector4d mass, dist;
    double mass_thigh = m[0] + m[1];
    double mass_shank = m[2] + m[3];
    double com_thigh = (s[0]*m[0] + (l[0]-s[1])*m[1]) / mass_thigh;
    double com_shank = (s[2]*m[2] + (l[1]-s[3])*m[3]) / mass_shank;
    mass << mass_thigh, mass_shank, mass_thigh, mass_shank;
    dist << com_thigh, com_shank, com_thigh, com_shank;
    x2FollowerState_->gravity.set_mass(mass);
    x2FollowerState_->gravity.set_dist(dist);
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
    //publishRequestedJointTorques();
    //publishJointReferencePositions();
    publishControullerOutputs();

    std_msgs::msg::Float64MultiArray arr;
    arr.data.resize(X2_NUM_JOINTS);
    memcpy(arr.data.data(), x2FollowerState_->strainGauge_, sizeof(double) * X2_NUM_JOINTS);
    strainGaugePublisher_->publish(arr);
    memcpy(arr.data.data(), x2FollowerState_->filteredGauge_, sizeof(double) * X2_NUM_JOINTS);
    filteredGaugePublisher_->publish(arr);
}

void X2MachineROS2::publishControullerOutputs(void){ 

    memcpy(controllerOutputsMsg_.friction.data(), x2FollowerState_->friction.output().data(), sizeof(double) * X2_NUM_JOINTS);
    memcpy(controllerOutputsMsg_.gravity.data(), x2FollowerState_->gravity.output().data(), sizeof(double) * X2_NUM_JOINTS);
    memcpy(controllerOutputsMsg_.total.data(), x2FollowerState_->getDesiredJointTorques().data(), sizeof(double) * X2_NUM_JOINTS);
    controllerOutputsMsg_.pd[0] = 69; // testing
    controllerOutputPublisher_->publish(controllerOutputsMsg_);
}


void X2MachineROS2::publishJointStates(void) {
    Eigen::VectorXd jointPositions = robot_->getPosition();
    Eigen::VectorXd jointVelocities = robot_->getVelocity();
    Eigen::VectorXd jointTorques = robot_->getTorque();
    //Eigen::VectorXd ankleEndpoints = kinHandler.fow_kin(robot_->getPosition());
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
    //ankleEndpoitsMsg_.left_x = ankleEndpoints(0);
    //ankleEndpoitsMsg_.left_y = ankleEndpoints(1);
    //ankleEndpoitsMsg_.right_x = ankleEndpoints(2);
    //ankleEndpoitsMsg_.right_y = ankleEndpoints(3);
    
    //ankleEndpointPublisher_->publish(ankleEndpoitsMsg_);
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
    //Tm kp, kd;
    //Tv alphaMax(gains->alpha_max.data()), alphaMin(gains->alpha_min.data());
    ////kp << gains->left_kp[0], gains->left_kp[1], 0 , 0,
    ////            gains->left_kp[2], gains->left_kp[3], 0, 0,
    ////            0, 0, gains->right_kp[0], gains->right_kp[1],
    ////            0, 0, gains->right_kp[2], gains->right_kp[3];
    
    ////kd << gains->left_kd[0], gains->left_kd[1], 0 , 0,
    ////            gains->left_kd[2], gains->left_kd[3], 0, 0,
    ////            0, 0, gains->right_kp[0], gains->right_kp[1],
    ////            0, 0, gains->right_kp[2], gains->right_kp[3];

    //kp << gains->left_kp[1], gains->left_kp[2], 0 , 0,
    //            gains->left_kp[3], gains->left_kp[4], 0, 0,
    //            0, 0, gains->right_kp[1], gains->right_kp[2],
    //            0, 0, gains->right_kp[3], gains->right_kp[4];

    //kd << gains->left_kd[1], gains->left_kd[2], 0 , 0,
    //            gains->left_kd[3], gains->left_kd[4], 0, 0,
    //            0, 0, gains->right_kp[1], gains->right_kp[2],
    //            0, 0, gains->right_kp[3], gains->right_kp[4];
    //x2FollowerState_->PDCntrl->set_gains(kp,kd);
    //x2FollowerState_->PDCntrl->set_alphas(alphaMin, alphaMax);
}

void X2MachineROS2::jointRefCallback(const sensor_msgs::msg::JointState::SharedPtr angles) {
    //Update the joint references in X2follower state
    //Tv jointRef(angles->position.data());
    //x2FollowerState_->desiredJointReferences_ = jointRef;
}

void X2MachineROS2::externalForceCallback(const x2_msgs::msg::External::SharedPtr ext){
    //Tv torque(ext->torque.data());
    //x2FollowerState_->ExtCntrl->set_external_torque(torque);
}

void X2MachineROS2::frictionForceCallback(const x2_msgs::msg::Friction::SharedPtr fric) {
    Tv b_static, b_viscous;
    memcpy(b_static.data(), fric->b_static.data(), sizeof(double) * X2_NUM_JOINTS);
    memcpy(b_viscous.data(), fric->b_viscous.data(), sizeof(double) * X2_NUM_JOINTS);
    x2FollowerState_->friction.set_static(b_static);
    x2FollowerState_->friction.set_viscous(b_viscous);
}

void X2MachineROS2::enablerCallback(const x2_msgs::msg::Enable::SharedPtr enable) {
    //TODO
}


void X2MachineROS2::corcParamCallback(const x2_msgs::msg::Corc::SharedPtr corcParams) {
    x2FollowerState_->rateLimit = corcParams->reference_limit;
    x2FollowerState_->maxTorqueLimit = corcParams->maximum_torque;
    x2FollowerState_->posReader.updateTrajectoryTime(corcParams->trajectory_period);
}
