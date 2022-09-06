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
    mode = TEST;
    state_ = STEP_UP;
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

    jointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    prevJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    prevJointReferences_  = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    jointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

    refPosPeriod = 5;
    rateLimit = 0.0;
    maxTorqueLimit = 0.0;

    PDCntrl = new ctrl::PDController<double, X2_NUM_JOINTS, 500>();
    ExtCntrl = new ctrl::ExternalController<double, X2_NUM_JOINTS>();
    FricCntrl = new ctrl::FrictionController<double, X2_NUM_JOINTS>();
    GravCntrl = new ctrl::GravityController<double, X2_NUM_JOINTS>();
    
    // controllers.insert(std::pair<cntrl, PDController<double, X2_NUM_JOINTS>*>(PD, PDCntrl));
    // controllers.insert(std::pair<cntrl, ExternalController<double, X2_NUM_JOINTS>*>(Ext, ExtCntrl));
    // controllers.insert(std::pair<cntrl, FrictionController<double, X2_NUM_JOINTS>*>(Fric, FricCntrl));
    // // Gravity to be implmented     
    controllers = {PDCntrl, ExtCntrl, FricCntrl, GravCntrl};
    posReader = LookupTable<double, X2_NUM_JOINTS>(1, 1);
    //Logging Sedtup
    dpos_logger =   spdlog::basic_logger_mt("dpos", "logs/posd_logs.log", true);
    pos_logger =   spdlog::basic_logger_mt("pos", "logs/pos_logs.log", true);
    dref_logger = spdlog::basic_logger_mt("dref", "logs/dref_logs.log", true);
    ref_logger = spdlog::basic_logger_mt("ref", "logs/ref_logs.log", true);
    p_logger =   spdlog::basic_logger_mt("p_out", "logs/p_logs.log", true);
    d_logger =   spdlog::basic_logger_mt("d_out", "logs/d_logs.log", true);
    torque_logger =   spdlog::basic_logger_mt("torque", "logs/pdout_logs.log", true);
    effort_logger =   spdlog::basic_logger_mt("effort", "logs/effort_logs.log", true);

    err_logger =   spdlog::basic_logger_mt("err", "logs/err_logs.log", true);
    spdlog::flush_every(std::chrono::seconds(2)); 

    butter.set_coeff_a({1,-1.7345,0.7658});
    butter.set_coeff_b({0.0078, 0.0157, 0.0078});
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
    currTrajProgress = 0;
    trajTime = 2;
}

void X2FollowerState::entry(void) {
    spdlog::info("Entered Follower State");
    posReader.readCSV(csvFileName);
    posReader.startTrajectory(robot_->getPosition(), 4);
    safetyFlag = false; //Initialise with no safetyfkag
    time0 = std::chrono::steady_clock::now();
    _Time_prev = time0;
}

void X2FollowerState::during(void) {

    // set maximum joint torque limits
    // TODO
    //Do not run if the safety flag is triggered
    if (safetyFlag) {
        return;
    }
    if (mode == IDLE) {
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);

    } else if (mode == GAIT){
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
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        // desiredJointTorques_ = PDCntrl->output();
        for(auto &cnt : controllers) {
            desiredJointTorques_ += cnt->output();

        }
        torqueLimiter(maxTorqueLimit);

        robot_->setTorque(desiredJointTorques_);
    } else if (mode == TEST) {

        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }
        clock_gettime(CLOCK_MONOTONIC, &currTime);
        double timeElapsed =  currTime.tv_sec - prevTime.tv_sec + (currTime.tv_nsec - prevTime.tv_nsec)/(1e9);
        desiredJointPositions_[0] = 0.2 * sin(timeElapsed/5 * 2 * M_PI);
        desiredJointPositions_[1] =0;
        desiredJointPositions_[2] = 0.2 * sin(timeElapsed/5 * 2 * M_PI);
        desiredJointPositions_[3] =0;

        jointPositions_ = robot_->getPosition();
        butter.filter(jointPositions_);
        jointPositions_ = butter.output();
        jointTorques_ = robot_->getTorque();
        PDCntrl->loop(desiredJointPositions_ - jointPositions_);
        //Logging
        auto dt = static_cast<double>(
            (std::chrono::steady_clock::now() - _Time_prev).count() / 1e9
        );
        pos_logger->info("{},{},{},{},{},{}", jointPositions_[0], jointPositions_[1], jointPositions_[2], jointPositions_[3], desiredJointPositions_[0], desiredJointPositions_[2]);
        ref_logger->info("{},{},{},{}", desiredJointPositions_[0], desiredJointPositions_[1], desiredJointPositions_[2], desiredJointPositions_[3]);

        dJointPositions_ = (jointPositions_ - prevJointPositions_) / dt;
        dJointReferences_ = (desiredJointPositions_ - prevJointReferences_) / dt;
        dpos_logger->info("{},{},{},{}", dJointPositions_[0], dJointPositions_[1], dJointPositions_[2], dJointPositions_[3]);
        dref_logger->info("{},{},{},{}", dJointReferences_[0], dJointReferences_[1], dJointReferences_[2], dJointReferences_[3]);

        p_logger->info("{},{},{},{}", PDCntrl->get_p()[0], PDCntrl->get_p()[1], PDCntrl->get_p()[2], PDCntrl->get_p()[3]);
        d_logger->info("{},{},{},{}", PDCntrl->get_d()[0], PDCntrl->get_d()[1], PDCntrl->get_d()[2], PDCntrl->get_d()[3]);
        torque_logger->info("{},{},{},{}", PDCntrl->output()[0], PDCntrl->output()[1], PDCntrl->output()[2], PDCntrl->output()[3]);
        effort_logger->info("{},{},{},{}", jointTorques_[0], jointTorques_[1], jointTorques_[2], jointTorques_[3]);
        err_logger->info("{},{},{},{}", PDCntrl->get_err_prev()[0],  PDCntrl->get_err_prev()[1],PDCntrl->get_err_prev()[2], PDCntrl->get_err_prev()[3]);

        prevJointPositions_ = jointPositions_;
        prevJointReferences_ = desiredJointPositions_;

        // std::cout << PDCntrl->get_kd() << std::endl;
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        for(auto &cnt : controllers) {
            desiredJointTorques_ += cnt->output();
        }
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
        spdlog::info("Completed Cycles: {}", posReader.getCycles());
     
    } else if (mode == TUNE) {
        // Mode for tuning PD controller for joint motors
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // switch between two set joint positions
        switch(state_) {
            case STEP_UP:
                desiredJointPositions_[LEFT_HIP] = deg2rad(0);
                desiredJointPositions_[LEFT_KNEE] = deg2rad(0);
                desiredJointPositions_[RIGHT_HIP] = deg2rad(0);
                desiredJointPositions_[RIGHT_KNEE] = deg2rad(0);

                if (!(t_count_ % (refPosPeriod * freq_))) {
                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
            case STEP_DOWN:
                desiredJointPositions_[LEFT_HIP] = deg2rad(-10);
                desiredJointPositions_[LEFT_KNEE] = deg2rad(-10);
                desiredJointPositions_[RIGHT_HIP] = deg2rad(-10);
                desiredJointPositions_[RIGHT_KNEE] = deg2rad(-10);

                if (!(t_count_ % (refPosPeriod * freq_))) {
                    state_ = STEP_DOWN;
                    t_count_ = 0;
                }
                break;
        }

        // increment callback counter
        t_count_++;
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

/**
 * @brief Checks that the torque does not change a significant amount (As this can cause unsafe jitter)
 * 
 * @return true 
 * @return false 
 * @todo Need to refine the actual value of this limit, it is currently chosen arbitrarily
 */
bool X2FollowerState::checkSafety() {

    double jerkLim = abs(maxTorqueLimit * 0.25 / freq_);
    for(int i = 0; i < desiredJointTorques_.size(); i++) {
        if(abs(desiredJointTorques_[i] - prevDesiredJointTorques_[i]) > jerkLim) {
            return true;
        }
    }
    return false;
}
