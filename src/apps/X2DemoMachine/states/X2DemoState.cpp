#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, const float updateT, const char *name) :
        State(m, name), robot_(exo), freq_(1 / updateT) {
    desiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    prevDesiredJointPositions_ = robot_->getPosition();
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    enableJoints = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    kTransperancy_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    amplitude_ = 0.0;
    period_ = 5.0;
    offset_ = 0.0;
    debugTorques = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
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

    if(controller_mode_ == 0){                                          // step torque controller 

        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {

            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // switch between two set joint positions
        switch(state_) {

            case STEP_UP:

                    desiredJointPositions_[0] = -deg2rad(20);
                    desiredJointPositions_[1] = -deg2rad(20);
                    desiredJointPositions_[2] = -deg2rad(20);
                    desiredJointPositions_[3] = -deg2rad(20);
                if (!(t_count_ % (5 * freq_))) {

                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:

                    desiredJointPositions_[0] = -deg2rad(10);
                    desiredJointPositions_[1] = -deg2rad(10);
                    desiredJointPositions_[2] = -deg2rad(10);
                    desiredJointPositions_[3] = -deg2rad(10);
                if (!(t_count_ % (5 * freq_))) {

                    state_ = STEP_UP;
                    t_count_ = 0;
                }
                break;
        }

        // make sure desiredJointPositions_ is velocity limited
        X2DemoState::vel_limiter(deg2rad(20)); // 20 deg/second velocity limit

        auto torques = jointControllers.loopc(desiredJointPositions_.data(), robot_->getPosition().data());

        for (std::size_t i = 0; i < torques.size(); i++) {
            if (torques[i] < -LIMIT_TORQUE) {
                spdlog::error("ERROR: Torque limit reached for joint {}", i);
                torques[i] = -LIMIT_TORQUE;
            }
			else if (torques[i] > LIMIT_TORQUE) {
                spdlog::error("ERROR: Torque limit reached for joint {}", i);
                torques[i] = LIMIT_TORQUE;
            }
        }

        desiredJointTorques_ << torques[0], torques[1], torques[2], torques[3];

        spdlog::info("OUTPUT: {}", desiredJointTorques_[1]);
        spdlog::info("DT    : {}", jointControllers.dtc());

        // add debug torques to all joints 
        desiredJointTorques_ += debugTorques;
        robot_->setTorque(desiredJointTorques_);
        t_count_++;
    } else if (controller_mode_ == 1) {                                 // sin torque
    
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {

            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count() / 1000.0;
        for(int joint = 0; joint < X2_NUM_JOINTS; joint++) {
            desiredJointTorques_[joint] = enableJoints[joint] * amplitude_ * sin(2.0 * M_PI / period_ * time);
        }

        // add debug torques to all joints
        desiredJointTorques_ += debugTorques;

        robot_->setTorque(desiredJointTorques_);
    } else if (controller_mode_ == 2) {                                 // no step

        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {

            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        desiredJointPositions_ << 0.0, 0.0, 0.0, 0.0;

        auto torques = jointControllers.loop(desiredJointPositions_.data(), robot_->getPosition().data());
        desiredJointTorques_ << torques[0], torques[1], torques[2], torques[3];

        // add debug torques to all joints
        desiredJointTorques_ += debugTorques;

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

void X2DemoState::vel_limiter(double limit) {

    auto dJointPositions = desiredJointPositions_ - prevDesiredJointPositions_;
    double maxJointPositionDelta = limit * freq_;

    for (int i = 0; i < dJointPositions.size(); i++) {

        if (dJointPositions[i] * freq_ > limit) {
            desiredJointPositions_[i] = prevDesiredJointPositions_[i] + maxJointPositionDelta;
        }
    } 

    prevDesiredJointPositions_ = desiredJointPositions_;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

Eigen::VectorXd &X2DemoState::getDesiredJointVelocities() {
    return desiredJointVelocities_;
}
