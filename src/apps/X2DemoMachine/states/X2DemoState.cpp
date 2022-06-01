#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, const float updateT, const char *name) :
        State(m, name), robot_(exo), freq_(1 / updateT) {
    desiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    prevDesiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesP_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesI_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesD_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    enableJoints = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    kTransperancy_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    amplitude_ = 0.0;
    period_ = 5.0;
    offset_ = 0.0;
    debugTorques = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    frictionCompensationTorques = Eigen::VectorXd::Zero(8);
    jointControllers.set_limit(-LIMIT_TORQUE, LIMIT_TORQUE);
    jointControllers[0].bind([](auto& Kp, auto& Ki, auto& Kd){});
    jointControllers[1].bind([](auto& Kp, auto& Ki, auto& Kd){});
    jointControllers[2].bind([](auto& Kp, auto& Ki, auto& Kd){});
    jointControllers[3].bind([](auto& Kp, auto& Ki, auto& Kd){});
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

    jointControllers.set_limit(-maxTorqueLimit, maxTorqueLimit);

    if(controller_mode_ == 0){                                          // all joints step torque controller 

        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {

            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // switch between two set joint positions
        switch(state_) {

            case STEP_UP:

                    desiredJointPositions_[0] = deg2rad(refPos1);
                    desiredJointPositions_[1] = deg2rad(refPos1);
                    desiredJointPositions_[2] = deg2rad(refPos1);
                    desiredJointPositions_[3] = deg2rad(refPos1);
                if (!(t_count_ % (refPosPeriod * freq_))) {

                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:

                    desiredJointPositions_[0] = deg2rad(refPos2);
                    desiredJointPositions_[1] = deg2rad(refPos2);
                    desiredJointPositions_[2] = deg2rad(refPos2);
                    desiredJointPositions_[3] = deg2rad(refPos2);
                if (!(t_count_ % (refPosPeriod * freq_))) {

                    state_ = STEP_UP;
                    t_count_ = 0;
                }
                break;
        }

        // make sure desiredJointPositions_ is velocity limited
        vel_limiter(deg2rad(rateLimit));

        auto torques = jointControllers.loop(desiredJointPositions_.data(), robot_->getPosition().data());

        desiredJointTorques_ << torques[0], torques[1], torques[2], torques[3];
        desiredJointTorquesP_ << jointControllers[0].p(), jointControllers[1].p(), jointControllers[2].p(), jointControllers[3].p();
        desiredJointTorquesD_ << jointControllers[0].d(), jointControllers[1].d(), jointControllers[2].d(), jointControllers[3].d();

        // add debug torques to all joints based on the torque direction being applied
        addDebugTorques(0);
        addDebugTorques(1);
        addDebugTorques(2);
        addDebugTorques(3);

        // add friction compenstation torques to all joints based on the torque direction being applied
        addFrictionCompensationTorques(0);
        addFrictionCompensationTorques(1);
        addFrictionCompensationTorques(2);
        addFrictionCompensationTorques(3);

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

        desiredJointTorquesP_ << 0, 0, 0, 0;
        desiredJointTorquesD_ << 0, 0, 0, 0;

        // add debug torques to all joints based on the torque direction being applied
        addDebugTorques(0);
        addDebugTorques(1);
        addDebugTorques(2);
        addDebugTorques(3);

        // add friction compenstation torques to all joints based on the torque direction being applied
        addFrictionCompensationTorques(0);
        addFrictionCompensationTorques(1);
        addFrictionCompensationTorques(2);
        addFrictionCompensationTorques(3);

        robot_->setTorque(desiredJointTorques_);
    } else if (controller_mode_ == 2) {                                 // no step

        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {

            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        desiredJointPositions_ << 0.0, 0.0, 0.0, 0.0;

        vel_limiter(deg2rad(rateLimit));

        auto torques = jointControllers.loop(desiredJointPositions_.data(), robot_->getPosition().data());
        desiredJointTorques_ << torques[0], torques[1], torques[2], torques[3];

        desiredJointTorquesP_ << jointControllers[0].p(), jointControllers[1].p(), jointControllers[2].p(), jointControllers[3].p();
        desiredJointTorquesD_ << jointControllers[0].d(), jointControllers[1].d(), jointControllers[2].d(), jointControllers[3].d();


        // add debug torques to all joints based on the torque direction being applied
        addDebugTorques(0);
        addDebugTorques(1);
        addDebugTorques(2);
        addDebugTorques(3);

        // add friction compenstation torques to all joints based on the torque direction being applied
        addFrictionCompensationTorques(0);
        addFrictionCompensationTorques(1);
        addFrictionCompensationTorques(2);
        addFrictionCompensationTorques(3);

        robot_->setTorque(desiredJointTorques_);
    } else if (controller_mode_ == 3) {                                // left hip with locked left knee control mode

        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {

            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // switch between two set joint positions
        switch(state_) {

            case STEP_UP:

                    desiredJointPositions_[0] = -deg2rad(20);
                    desiredJointPositions_[1] = -deg2rad(0);
                    desiredJointPositions_[2] = -deg2rad(20);
                    desiredJointPositions_[3] = -deg2rad(20);
                if (!(t_count_ % (5 * freq_))) {

                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:

                    desiredJointPositions_[0] = -deg2rad(10);
                    desiredJointPositions_[1] = -deg2rad(0);
                    desiredJointPositions_[2] = -deg2rad(10);
                    desiredJointPositions_[3] = -deg2rad(10);
                if (!(t_count_ % (5 * freq_))) {

                    state_ = STEP_UP;
                    t_count_ = 0;
                }
                break;
        }

        vel_limiter(deg2rad(rateLimit));

        auto torques = jointControllers.loop(desiredJointPositions_.data(), robot_->getPosition().data());

        desiredJointTorques_ << torques[0], torques[1], torques[2], torques[3];
        desiredJointTorquesP_ << jointControllers[0].p(), jointControllers[1].p(), jointControllers[2].p(), jointControllers[3].p();
        desiredJointTorquesD_ << jointControllers[0].d(), jointControllers[1].d(), jointControllers[2].d(), jointControllers[3].d();

        // add debug torques to all joints based on the torque direction being applied
        addDebugTorques(0);
        addDebugTorques(1);
        addDebugTorques(2);
        addDebugTorques(3);

        // add friction compenstation torques to all joints based on the torque direction being applied
        addFrictionCompensationTorques(0);
        addFrictionCompensationTorques(1);
        addFrictionCompensationTorques(2);
        addFrictionCompensationTorques(3);

        robot_->setTorque(desiredJointTorques_);
        t_count_++;
    } else if (controller_mode_ == 4) {                                  // right hip with right knee locked control mode

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
                    desiredJointPositions_[3] = -deg2rad(0);
                if (!(t_count_ % (5 * freq_))) {

                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:

                    desiredJointPositions_[0] = -deg2rad(10);
                    desiredJointPositions_[1] = -deg2rad(10);
                    desiredJointPositions_[2] = -deg2rad(10);
                    desiredJointPositions_[3] = -deg2rad(0);
                if (!(t_count_ % (5 * freq_))) {

                    state_ = STEP_UP;
                    t_count_ = 0;
                }
                break;
        }

        vel_limiter(deg2rad(rateLimit));

        auto torques = jointControllers.loop(desiredJointPositions_.data(), robot_->getPosition().data());

        desiredJointTorques_ << torques[0], torques[1], torques[2], torques[3];
        desiredJointTorquesP_ << jointControllers[0].p(), jointControllers[1].p(), jointControllers[2].p(), jointControllers[3].p();
        desiredJointTorquesD_ << jointControllers[0].d(), jointControllers[1].d(), jointControllers[2].d(), jointControllers[3].d();

        // add debug torques to all joints based on the torque direction being applied
        addDebugTorques(0);
        addDebugTorques(1);
        addDebugTorques(2);
        addDebugTorques(3);

        // add friction compenstation torques to all joints based on the torque direction being applied
        addFrictionCompensationTorques(0);
        addFrictionCompensationTorques(1);
        addFrictionCompensationTorques(2);
        addFrictionCompensationTorques(3);

        robot_->setTorque(desiredJointTorques_);
        t_count_++;
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
    double maxJointPositionDelta = abs(limit / freq_);

    double newDesiredJointPosition = 0;
    for (int i = 0; i < dJointPositions.size(); i++) {

        if (abs(dJointPositions[i]) > maxJointPositionDelta) {

            if (dJointPositions[i] > 0) {
                newDesiredJointPosition = prevDesiredJointPositions_[i] + maxJointPositionDelta; 
            } else {
                newDesiredJointPosition = prevDesiredJointPositions_[i] - maxJointPositionDelta;
            }

            desiredJointPositions_[i] = newDesiredJointPosition;
            prevDesiredJointPositions_[i] = newDesiredJointPosition;
        }

    } 
}

void X2DemoState::addDebugTorques(int joint) {

    // account for torque sign
    auto externalTorque = debugTorques[joint];

    if (abs(desiredJointTorques_[joint] + externalTorque) < maxTorqueLimit) {
        desiredJointTorques_[joint] += externalTorque;
    } else if (desiredJointTorques_[joint] + externalTorque > 0) {
        desiredJointTorques_[joint] = maxTorqueLimit;
    } else {
        desiredJointTorques_[joint] = -maxTorqueLimit;
    }
}

void X2DemoState::addFrictionCompensationTorques(int joint) {

    // account for torque sign
    auto externalTorquePos = frictionCompensationTorques[2 * joint]; 
    auto externalTorqueNeg = frictionCompensationTorques[2 * joint + 1];

    if (desiredJointTorques_[joint] == 0) {
        externalTorquePos = 0;
        externalTorqueNeg = 0;
    }

    if (desiredJointTorques_[joint] > 0) {

        if (abs(desiredJointTorques_[joint] + externalTorquePos) < maxTorqueLimit) {
            desiredJointTorques_[joint] += externalTorquePos;
        } else if (desiredJointTorques_[joint] + externalTorquePos > 0) {
            desiredJointTorques_[joint] = maxTorqueLimit;
        } else {
            desiredJointTorques_[joint] = -maxTorqueLimit;
        }
    } else if (desiredJointTorques_[joint] < 0) {

        if (abs(desiredJointTorques_[joint] + externalTorqueNeg) < maxTorqueLimit) {
            desiredJointTorques_[joint] += externalTorqueNeg;
        } else if (desiredJointTorques_[joint] + externalTorqueNeg > 0) {
            desiredJointTorques_[joint] = maxTorqueLimit;
        } else {
            desiredJointTorques_[joint] = -maxTorqueLimit;
        }
    }
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

Eigen::VectorXd &X2DemoState::getDesiredJointVelocities() {
    return desiredJointVelocities_;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorquesPSplit() {
    return desiredJointTorquesP_;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorquesISplit() {
    return desiredJointTorquesI_;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorquesDSplit() {
    return desiredJointTorquesD_;
}

Eigen::VectorXd &X2DemoState::getDesiredJointPositions() {
    return desiredJointPositions_;
}