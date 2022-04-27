#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, const float updateT, const char *name) :
        State(m, name), robot_(exo), freq_(1 / (updateT / 1000)) {
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    enableJoints = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    kTransperancy_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    amplitude_ = 0.0;
    period_ = 5.0;
    offset_ = 0.0;

    kd = 0;
    kp = 0;
    pd(kp, kd);
    debug_torque = 0.0;
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

//#ifndef SIM
//    // GREEN BUTTON IS THE DEAD MAN SWITCH --> if it is not pressed, all motor torques are set to 0. Except controller 2 which sets 0 velocity
//    if(robot_->getButtonValue(ButtonColor::GREEN) == 0 && controller_mode_ !=2){
//        if(robot_->getControlMode()!=CM_TORQUE_CONTROL) robot_->initTorqueControl();
//        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
//        robot_->setTorque(desiredJointTorques_);
//
//        return;
//    }
//#endif

    // update controller gains
    pd.Kp = kp;
    pd.Kd = kd;
    
    if(controller_mode_ == 0){                                          // step torque controller 

        if (robot_->getControlMode()!=CM_TORQUE_CONTROL) {

            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

		Eigen::VectorXd desiredJointPositions_(X2_NUM_JOINTS);

        // switch between 0 and non-zero joint positions
        switch(state_) {

            case STEP_UP:

                    desiredJointPositions_[1] = -deg2rad(20);
                    desiredJointPositions_[3] = -deg2rad(20);
                if (!(t_count_ % (5 * freq_))) {

                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:

                if (!(t_count_ % (5 * freq_))) {

                    state_ = STEP_UP;
                    t_count_ = 0;
                }
                break;
        }

        desiredJointTorques_ = pd.loop(desiredJointPositions_, robot_->getPosition());

        // added debug torque to left knee
        desiredJointTorques_[1] += debug_torque;

        robot_->setTorque(desiredJointTorques_);
        t_count_++;
        spdlog::info("Torque {0} {1} {2} {3}",
            desiredJointTorques_[0],
            desiredJointTorques_[1],
            desiredJointTorques_[2],
            desiredJointTorques_[3]
        );
    } else if (controller_mode_ == 1) {                                 // sin torque
    
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {

            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count() / 1000.0;
        for(int joint = 0; joint < X2_NUM_JOINTS; joint++) {
            desiredJointTorques_[joint] = enableJoints[joint] * amplitude_ * sin(2.0 * M_PI / period_ * time);
        }

        // add debug torque to left knee
        desiredJointTorques_[1] += debug_torque;

        robot_->setTorque(desiredJointTorques_);
    } else if (controller_mode_ == 2) {                                 // no step

        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {

            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

		Eigen::VectorXd desiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        desiredJointTorques_ = pd.control_loop(desiredJointPositions_, robot_->getPosition());

        // add debug torque to left knee
        desiredJointTorques_[1] += debug_torque;

        robot_->setTorque(desiredJointTorques_);
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
