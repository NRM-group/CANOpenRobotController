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
    mode = IK_GAIT;
    desiredJointReferences_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    prevDesiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    prevDesiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesP_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesI_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesD_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    actualDesiredJointPositions_= Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    startJointPositions_ = robot_->getPosition();

    refPos1 = 0;
    refPos2 = 0;
    refPosPeriod = 5;
    rateLimit = 0.0;
    maxTorqueLimit = 0.0;

    PDCntrl = new PDController<double, X2_NUM_JOINTS>();
    ExtCntrl = new ExternalController<double, X2_NUM_JOINTS>();
    FricCntrl = new FrictionController<double, X2_NUM_JOINTS>();
    
    // controllers.insert(std::pair<cntrl, PDController<double, X2_NUM_JOINTS>*>(PD, PDCntrl));
    // controllers.insert(std::pair<cntrl, ExternalController<double, X2_NUM_JOINTS>*>(Ext, ExtCntrl));
    // controllers.insert(std::pair<cntrl, FrictionController<double, X2_NUM_JOINTS>*>(Fric, FricCntrl));
    // // Gravity to be implmented     
    controllers = {PDCntrl, ExtCntrl, FricCntrl};
    posReader = LookupTable(X2_NUM_JOINTS,1, 1);
    
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
    currTrajProgress = 0;
    gaitIndex = 0;
    trajTime = 2;
}

void X2FollowerState::entry(void) {
    spdlog::info("Entered Follower State");
    posReader.readCSV(csvFileName);
    safetyFlag = false; //Initialise with no safetyfkag
    time0 = std::chrono::steady_clock::now();
}

void X2FollowerState::during(void) {

    // set maximum joint torque limits
    // TODO
    //Do not run if the safety flag is triggered
    if (safetyFlag) {
        return;
    }
    if (mode == GAIT){
        // switch motor control mode to torque control
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        desiredJointPositions_ = posReader.getNextPos();

        for(int j = 0; j < X2_NUM_JOINTS; j++) {
            
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
        actualDesiredJointPositions_ = desiredJointReferences_;
        rateLimiter(deg2rad(rateLimit));
        PDCntrl->loop(desiredJointPositions_, robot_->getPosition());
        // desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        desiredJointTorques_ = PDCntrl->output();
        // for(auto &cnt : controllers) {
        //     desiredJointTorques_ += cnt->output();

        // }
        torqueLimiter(maxTorqueLimit);

        robot_->setTorque(desiredJointTorques_);
    } else if (mode == IK) {

        trajTime = 1;
        if (robot_->getControlMode()!=CM_TORQUE_CONTROL) robot_->initTorqueControl();
        
        desiredJointPositions_ = desiredJointReferences_;
        rateLimiter(deg2rad(rateLimit));
        PDCntrl->loop(actualDesiredJointPositions_, robot_->getPosition());
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        for(auto &cnt : controllers) {
            //Change to be the max torque if greater than
            desiredJointTorques_ += cnt->output();

        }
        //Torque limiter function
        torqueLimiter(maxTorqueLimit);
        robot_->setTorque(desiredJointTorques_);

    } else if (mode == IK_GAIT) {
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initialised Torque Control");
        }
        Eigen::VectorXd computedState = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        computedState = posReader.getNextPos();
         
        Eigen::VectorXd FKcoords = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        FKcoords = kinHandler.fow_kin(computedState);
        desiredJointPositions_ = kinHandler.inv_kin(FKcoords);
        for(int j = 0; j < X2_NUM_JOINTS; j++) {
            
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

        rateLimiter(deg2rad(rateLimit));
        PDCntrl->loop(desiredJointPositions_, robot_->getPosition());
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        for(auto &cnt : controllers) {
            //Change to be the max torque if greater than
            desiredJointTorques_ += cnt->output();

        }
        //Torque limiter function
        torqueLimiter(maxTorqueLimit);
        robot_->setTorque(desiredJointTorques_);

     
    }
}

void X2FollowerState::exit(void) {
    // setting 0 torque for safety.
    robot_->initTorqueControl();
    robot_->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
}

void X2FollowerState::torqueLimiter(double limit) {
    for (int i = 0; i < desiredJointTorques_.size() ; i ++) {
        if (desiredJointTorques_[i] > limit) {
            desiredJointTorques_[i] = limit;
        } else if (desiredJointTorques_[i] < -limit) {
            desiredJointTorques_[i] = -limit;
        }
    }
    prevDesiredJointTorques_ = desiredJointTorques_;
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

            actualDesiredJointPositions_[i] = newDesiredJointPosition;
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

Eigen::VectorXd &X2FollowerState::getActualDesiredJointPositions() {
    return actualDesiredJointPositions_;
}

bool X2FollowerState::checkSafety() {
    //Change to differences
    //Check that the robot's torque has not exceeded the limits

    double jerkLim = abs(maxTorqueLimit * 0.25 / freq_);
    for(int i = 0; i < desiredJointTorques_.size(); i++) {
        if(abs(desiredJointTorques_[i] - prevDesiredJointTorques_[i]) > jerkLim) {
            return true;
        }
    }
    return false;
}
