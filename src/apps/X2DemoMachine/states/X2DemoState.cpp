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
    refPos1 = 0;
    refPos2 = 0;
    refPosPeriod = 5;
    rateLimit = 0.0;

    debugTorques = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    frictionCompensationTorques = Eigen::VectorXd::Zero(8);

    jointControllers[0].set_limit({-LIMIT_TORQUE, -LIMIT_TORQUE}, {LIMIT_TORQUE, LIMIT_TORQUE});
    jointControllers[1].set_limit({-LIMIT_TORQUE, -LIMIT_TORQUE}, {LIMIT_TORQUE, LIMIT_TORQUE});

    posReader = LookupTable(X2_NUM_JOINTS);
    posReader.readCSV("/home/bigbird/catkin_ws/src/CANOpenRobotController/src/apps/X2DemoMachine/gaits/GaitTrajectory_220602_1605.csv");
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
    currTrajProgress = 0;
    gaitIndex = 0;
    trajTime = 2;
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

    // set maximum joint torque limits
    jointControllers[0].set_limit({-maxTorqueLimit, -maxTorqueLimit}, {maxTorqueLimit, maxTorqueLimit});
    jointControllers[1].set_limit({-maxTorqueLimit, -maxTorqueLimit}, {maxTorqueLimit, maxTorqueLimit});

    if(controller_mode_ == 0){                                          // all joints step torque controller 

        // switch motor control mode to torque control mode
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // switch between two set joint positions
        switch(state_) {
            case STEP_UP:
                    desiredJointPositions_[LEFT_HIP] = deg2rad(refPos1);
                    desiredJointPositions_[LEFT_KNEE] = deg2rad(refPos1);
                    desiredJointPositions_[RIGHT_HIP] = deg2rad(refPos1);
                    desiredJointPositions_[RIGHT_KNEE] = deg2rad(refPos1);

                if (!(t_count_ % (refPosPeriod * freq_))) {
                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:
                    desiredJointPositions_[LEFT_HIP] = deg2rad(refPos2);
                    desiredJointPositions_[LEFT_KNEE] = deg2rad(refPos2);
                    desiredJointPositions_[RIGHT_HIP] = deg2rad(refPos2);
                    desiredJointPositions_[RIGHT_KNEE] = deg2rad(refPos2);

                if (!(t_count_ % (refPosPeriod * freq_))) {
                    state_ = STEP_UP;
                    t_count_ = 0;
                }
                break;
        }

        // increment callback counter
        t_count_++;
    } else if (controller_mode_ == 1) {                                // left hip with locked left knee control mode

        // change motor control mode to torque control
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // switch between two set joint positions
        switch(state_) {
            case STEP_UP:
                    desiredJointPositions_[LEFT_HIP] = deg2rad(refPos1);
                    desiredJointPositions_[LEFT_KNEE] = deg2rad(0);
                    desiredJointPositions_[RIGHT_HIP] = deg2rad(0);
                    desiredJointPositions_[RIGHT_KNEE] = deg2rad(0);

                if (!(t_count_ % (refPosPeriod * freq_))) {
                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:
                    desiredJointPositions_[LEFT_HIP] = deg2rad(refPos2);
                    desiredJointPositions_[LEFT_KNEE] = deg2rad(0);
                    desiredJointPositions_[RIGHT_HIP] = deg2rad(0);
                    desiredJointPositions_[RIGHT_KNEE] = deg2rad(0);

                if (!(t_count_ % (refPosPeriod * freq_))) {
                    state_ = STEP_UP;
                    t_count_ = 0;
                }
                break;
        }

        // increment callback counter 
        t_count_++;
    } else if (controller_mode_ == 2) {                                  // right hip with right knee locked control mode

        // switch motor control mode to torque control
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // switch between two set joint positions
        switch(state_) {
            case STEP_UP:
                    desiredJointPositions_[LEFT_HIP] = deg2rad(0);
                    desiredJointPositions_[LEFT_KNEE] = deg2rad(0);
                    desiredJointPositions_[RIGHT_HIP] = deg2rad(refPos1);
                    desiredJointPositions_[RIGHT_KNEE] = deg2rad(0);

                if (!(t_count_ % (refPosPeriod * freq_))) {
                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:
                    desiredJointPositions_[LEFT_HIP] = deg2rad(0);
                    desiredJointPositions_[LEFT_KNEE] = deg2rad(0);
                    desiredJointPositions_[RIGHT_HIP] = deg2rad(refPos2);
                    desiredJointPositions_[RIGHT_KNEE] = deg2rad(0);

                if (!(t_count_ % (refPosPeriod * freq_))) {
                    state_ = STEP_UP;
                    t_count_ = 0;
                }
                break;
        }

        // increment callback counter
        t_count_++;
    } else if (controller_mode_ == 3) {                                 // custom trajectory following mode

        // switch motor control mode to torque control
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        timespec currTime;
        clock_gettime(CLOCK_MONOTONIC, &currTime);

        double timeElapsed = currTime.tv_sec - prevTime.tv_sec + (currTime.tv_nsec - prevTime.tv_nsec) / 1e9;
        prevTime = currTime;
        currTrajProgress += timeElapsed; 
        double progress = currTrajProgress / trajTime;
        trajTime = period_;

        int trajIndexes = 1;
        Eigen::VectorXd start(4);
        Eigen::VectorXd end(4);

        if(progress >= 1) {
            //When you have finished this linear point, move on to the next stage
            currTrajProgress = 0;
            gaitIndex += trajIndexes;
            return;
        }

        for(int j = 0; j < X2_NUM_JOINTS; j++) {
            
            start[j] = posReader.getPosition(j, gaitIndex);
            end[j] = posReader.getPosition(j, gaitIndex + trajIndexes);
            desiredJointPositions_[j] = (start[j] + progress * (end[j] - start[j]));

            if (j == LEFT_HIP || j == RIGHT_HIP) {
                // check hip bounds
                if (desiredJointPositions_[j] > deg2rad(120)) {
                    desiredJointPositions_[j] = deg2rad(120);
                } else if (desiredJointPositions_[j] < -deg2rad(40)) {
                    desiredJointPositions_[j] = -deg2rad(40);
                }
            } else if (j == LEFT_KNEE || j == RIGHT_KNEE) {
                // check knee bounds
                if (desiredJointPositions_[j] < -deg2rad(120)) {
                    desiredJointPositions_[j] = -deg2rad(120);
                } else if (desiredJointPositions_[j] > 0) {
                    desiredJointPositions_[j] = 0;
                }
            }
        }
    }

    /* calculate required joint torques */

    // limit the desiredJointPositions_ delta from previous callback
    vel_limiter(deg2rad(rateLimit));

    // obtain required joint torques from control loop to reach desiredJointPositions_
    desiredJointTorques_ = jointControllers.loop(desiredJointPositions_, robot_->getPosition());

    // store control loop torques for future use
    desiredJointTorquesP_ = jointControllers.get_p();
    desiredJointTorquesD_ = jointControllers.get_d();

    // add debug torques and friction compensation torques to all joints based on the torque direction being applied
    for (size_t i = 0; i < X2_NUM_JOINTS; i++) {
        addDebugTorques(i);
        addFrictionCompensationTorques(i);
    }

    // update motor torques to required values 
    robot_->setTorque(desiredJointTorques_);
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