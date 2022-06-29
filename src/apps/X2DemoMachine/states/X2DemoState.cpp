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

    posReader = LookupTable(X2_NUM_JOINTS);
    posReader.readCSV("/home/bigbird/catkin_ws/src/CANOpenRobotController/src/apps/X2DemoMachine/gaits/GaitTrajectory_220602_1605.csv");
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
    currTrajProgress = 0;
    gaitIndex = 0;
    trajTime = 2;

    Eigen::Matrix<double, 2, 2> p_gains;
    Eigen::Matrix<double, 2, 2> d_gains;
    Eigen::Matrix<double, 2, 1> lengths;
    Eigen::Matrix<double, 10, 1> learning_rate; 

    // single length hip and shank lengths
    lengths << 0.39, 0.40;

    // AFFC learning rates
    learning_rate << 10, 200, 1, 1, 10, 1, 50, 1, 20, 1;
    learning_rate *= 2e-6;

    // One leg low PD controller gains
    p_gains(0, 0) = 450.0 * 0.1;
    p_gains(1, 1) = 450.0 * 0.1;
    d_gains(0, 0) = sqrt(p_gains(0, 0));
    d_gains(1, 1) = sqrt(p_gains(1, 1));

    // setup AFFC controller and set Criterion 1 and Criterion 2
    affc = new AdaptiveController<double, 2, 50>(lengths, learning_rate, p_gains, d_gains);
    affc->set_criterions(2, 0.5);

    // unknown parameter logger
    lambda_logger = spdlog::basic_logger_mt("lambda_logger", "logs/affc_lambdas.log");
    tracking_error_logger = spdlog::basic_logger_mt("tracking_error_logger", "logs/affc_tracking_error.log");
}

X2DemoState::~X2DemoState(void) {
   delete affc; 
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

    if(controller_mode_ == 0){
        // AFFC Controller Mode

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
        trajTime = 0.01;

        std::size_t trajIndexes = 1;
        Eigen::VectorXd start(4);
        Eigen::VectorXd end(4);

        if (progress >= 1) {
            // when you have finished this linear point, move onto the next stage
            currTrajProgress = 0;
            gaitIndex += trajIndexes;
            return;
        }

        // we only want to loop through one leg at a time 
        for(int j = 0; j < X2_NUM_JOINTS / 2; j++) {
            
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
        vel_limiter(deg2rad(rateLimit));

        // check if the AFFC algorithm is finished
        if (affc->is_finished()) {
            spdlog::info("AFFC is finished");
            return;
        }

        // iterate the AFFC algorithm once
        if (posReader.isTrajectoryFinished(gaitIndex)) {
            // a complete period of the trajectory has been completed
            affc->loop(desiredJointPositions_, robot_->getPosition(), true);

            // log unknown parameters and tracking error over iterations
            Eigen::Matrix<double, 10, 1> lamdas = affc->peek_learned_params();
            Eigen::Matrix<double, 2, 1> tracking_err = affc->peek_tracking_error();
            lambda_logger->info("{},{},{},{},{},{},{},{},{},{}", 
                                lamdas(0, 0),
                                lamdas(1, 0),
                                lamdas(2, 0),
                                lamdas(3, 0),
                                lamdas(4, 0),
                                lamdas(5, 0),
                                lamdas(6, 0),
                                lamdas(7, 0),
                                lamdas(8, 0),
                                lamdas(9, 0)
            );
            tracking_error_logger->info("{}, {}", tracking_err(0, 0), tracking_err(1, 0));
        } else {
            affc->loop(desiredJointPositions_, robot_->getPosition(), false);
        }

        // obtain required joint torques from AFFC control loop to reach desiredJointPositions_
        desiredJointTorques_ = affc->output();

        // limit torques
        torque_limiter(-80.0, 80.0);

        // update motor torques to required values 
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

void X2DemoState::vel_limiter(const double limit) {
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

void X2DemoState::torque_limiter(const double lower_limit, const double upper_limit) {
    for (std::size_t i = 0; i < X2_NUM_JOINTS; i++) {
        if (abs(desiredJointTorques_[i]) > maxTorqueLimit) {
            if (desiredJointTorques_[i] > 0) {
                desiredJointTorques_[i] = upper_limit;
            } else {
                desiredJointTorques_[i] = lower_limit;
            }
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