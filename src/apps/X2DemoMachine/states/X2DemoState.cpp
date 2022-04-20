#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, const char *name) :
        State(m, name), robot_(exo) {
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    enableJoints = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    kTransperancy_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    amplitude_ = 0.0;
    period_ = 5.0;
    offset_ = 0.0;

    ctrl.init(X2_NUM_JOINTS, 1.0);
}

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    // set dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&X2DemoState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    time0 = std::chrono::steady_clock::now();

}

void X2DemoState::during(void) {

#ifndef SIM
    // GREEN BUTTON IS THE DEAD MAN SWITCH --> if it is not pressed, all motor torques are set to 0. Except controller 2 which sets 0 velocity
    if(robot_->getButtonValue(ButtonColor::GREEN) == 0 && controller_mode_ !=2){
        if(robot_->getControlMode()!=CM_TORQUE_CONTROL) robot_->initTorqueControl();
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);

        return;
    }
#endif

    if(controller_mode_ == 1){ // custom torque controller 
        // if(robot_->getControlMode()!=CM_TORQUE_CONTROL) robot_->initTorqueControl();

        // desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        // Eigen::VectorXd desired(4);
        // ctrl.set_pd_gains(kp, kd);
        // desiredJointTorques_ = ctrl.control_loop(robot_->getPosition(), desired);
        // robot_->setTorque(desiredJointTorques_);

        if (robot_->getControlMode()!=CM_POSITION_CONTROL) {
            robot_->initPositionControl();
            robot_->setPosControlContinuousProfile(true);
            spdlog::info("Initalised Position Control Mode");
        }

        Eigen::VectorXd desiredJointPositions_(X2_NUM_JOINTS);
        // switch between 0 and non-zero joint positions
        switch(state_) {
            case STEP_UP:
                desiredJointPositions_ << 0, -deg2rad(20), 0, 0;
                if (!(t_count_ % freq_)) {
                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:
                desiredJointPositions_ << 0, 0, 0, 0;
                if (!(t_count_ % freq_)) {
                    state_ = STEP_UP;
                    t_count_ = 0;
                }
                break;
        }

        t_count_++;
        robot_->setPosition(desiredJointPositions_);
    }
}

void X2DemoState::exit(void) {
    robot_->initTorqueControl();
    // setting 0 torque for safety.
    robot_->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
    std::cout << "Example State Exited" << std::endl;
}

void X2DemoState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {

    controller_mode_ = config.controller_mode;

    enableJoints[0] = config.left_hip;
    enableJoints[1] = config.left_knee;
    enableJoints[2] = config.right_hip;
    enableJoints[3] = config.right_knee;

    kTransperancy_[1] = config.k_left_hip;
    kTransperancy_[2] = config.k_left_knee;
    kTransperancy_[3] = config.k_right_hip;
    kTransperancy_[4] = config.k_right_knee;

    amplitude_ = config.Amplitude;
    period_ = config.Period;
    offset_ = config.offset;

    robot_->setJointVelDerivativeCutOffFrequency(config.acc_deriv_cutoff);
    robot_->setBackpackVelDerivativeCutOffFrequency(config.backpack_deriv_cutoff);
    robot_->setDynamicParametersCutOffFrequency(config.g_cutoff);

    if(controller_mode_ == 4 || controller_mode_ == 5) time0 = std::chrono::steady_clock::now();

    return;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

Eigen::VectorXd &X2DemoState::getDesiredJointVelocities() {
    return desiredJointVelocities_;
}