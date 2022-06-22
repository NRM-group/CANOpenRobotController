/**
 * @file X2FollowerState.cpp
 * @author nilp amin (nilpamin2@gmail.com.au)
 * @brief Custom state which allows X2 to follow custom gait trajectories 
 * @version 0.1
 * @date 2022-06-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "X2FollowerState.h"

X2FollowerState::X2FollowerState(StateMachine* m, X2Robot* exo, const float updateT, const char* name) :
        State(m, name), robot_(exo), freq_(1 / updateT) 
{
    desiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    prevDesiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesP_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesI_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesD_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

    refPos1 = 0;
    refPos2 = 0;
    refPosPeriod = 5;
    rateLimit = 0.0;

    debugTorques = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    frictionCompensationTorques = Eigen::VectorXd::Zero(2 * X2_NUM_JOINTS);

    jointControllers.left().set_limit({-LIMIT_TORQUE, -LIMIT_TORQUE}, {LIMIT_TORQUE, LIMIT_TORQUE});
    jointControllers.right().set_limit({-LIMIT_TORQUE, -LIMIT_TORQUE}, {LIMIT_TORQUE, LIMIT_TORQUE});

    posReader = LookupTable(X2_NUM_JOINTS);
    
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
    currTrajProgress = 0;
    gaitIndex = 0;
    trajTime = 2;
}

void X2FollowerState::entry(void) {
    spdlog::info("Entered Follower State");
    posReader.readCSV(csvFileName);
    time0 = std::chrono::steady_clock::now();
}

void X2FollowerState::during(void) {

    // set maximum joint torque limits
    jointControllers.left().set_limit({-maxTorqueLimit, -maxTorqueLimit}, {maxTorqueLimit, maxTorqueLimit});
    jointControllers.right().set_limit({-maxTorqueLimit, -maxTorqueLimit}, {maxTorqueLimit, maxTorqueLimit});

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
    trajTime = 0.05;

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

    // limit the desiredJointPositions_ delta from previous callback
    rateLimiter(deg2rad(rateLimit));

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
    spdlog::info("{} {} {} {}", desiredJointTorques_[0], desiredJointTorques_[1], desiredJointTorques_[2], desiredJointTorques_[3]);
    spdlog::info("{} {} {} {}", desiredJointPositions_[0], desiredJointPositions_[1], desiredJointPositions_[2], desiredJointPositions_[3]);
    robot_->setTorque(desiredJointTorques_);
}

void X2FollowerState::exit(void) {
    // setting 0 torque for safety.
    robot_->initTorqueControl();
    robot_->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
}

void X2FollowerState::rateLimiter(double limit) {

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

void X2FollowerState::addDebugTorques(int joint) {
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

void X2FollowerState::addFrictionCompensationTorques(int joint) {
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

Eigen::VectorXd &X2FollowerState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

Eigen::VectorXd &X2FollowerState::getDesiredJointVelocities() {
    return desiredJointVelocities_;
}

Eigen::VectorXd &X2FollowerState::getDesiredJointTorquesPSplit() {
    return desiredJointTorquesP_;
}

Eigen::VectorXd &X2FollowerState::getDesiredJointTorquesISplit() {
    return desiredJointTorquesI_;
}

Eigen::VectorXd &X2FollowerState::getDesiredJointTorquesDSplit() {
    return desiredJointTorquesD_;
}

Eigen::VectorXd &X2FollowerState::getDesiredJointPositions() {
    return desiredJointPositions_;
}