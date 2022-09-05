#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, const float updateT, const char *name) :
        State(m, name), robot_(exo), freq_(1 / updateT) {

    desiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    prevDesiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointAccelerations_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesP_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesI_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorquesD_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    enableJoints = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    affcFbTorque = Eigen::MatrixXd::Zero(2, 2);

    kTransperancy_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    amplitude_ = 0.0;
    period_ = 5.0;
    offset_ = 0.0;
    rateLimit = 30;
    maxTorqueLimit = 80;
    period_counter_ = 1;

    debugTorques = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    frictionCompensationTorques = Eigen::VectorXd::Zero(8);

    completed_cycles_ = 0;
    posReader_ = LookupTable<double, X2_NUM_JOINTS>(4);
    posReader_.readCSV("/home/bigbird/catkin_ws/src/CANOpenRobotController/lib/trajectorylib/gaits/walking.csv");
    posReader_.startTrajectory(robot_->getPosition(), 0.1);

    Eigen::Matrix<double, 4, 4> pgains;
    Eigen::Matrix<double, 4, 4> dgains;

    pgains <<  450, 0, 0, 0, 
                0, 450, 0, 0, 
                0, 0, 450, 0, 
                0, 0, 0, 450;
    dgains <<  42, 0, 0, 0,
                0, 40, 0, 0,
                0, 0, 42, 0,
                0, 0, 0, 40;

    pdController.set_gains(pgains, dgains);
    pdController.set_alphas({2, 1.9, 2, 1.9}, {2.2, 2, 2.2, 2});

    lowPass.set_coeff_a({1, 2, 1});
    lowPass.set_coeff_b({251.011, 455.255, -208.244});

    Eigen::Matrix<double, 2, 2> p_gains = Eigen::Matrix<double, 2, 2>::Zero();
    Eigen::Matrix<double, 2, 2> d_gains = Eigen::Matrix<double, 2, 2>::Zero();
    Eigen::Matrix<double, 2, 1> lengths = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 16, 1> learning_rate = Eigen::Matrix<double, 16, 1>::Zero(); 
    Eigen::Matrix<double, 16, 1> inital_guess = Eigen::Matrix<double, 16, 1>::Zero();

    // single length hip and shank lengths
    lengths << 0.4, 0.4;

    // AFFC learning rates
    // learning_rate << 30, 20, 1, 1, 10, 1, 100, 1, 20, 1;
    // learning_rate << 10, 200, 1, 1, 10, 1, 50, 1, 20, 1;
    learning_rate << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    // learning_rate << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    learning_rate *= 2e-5;

    // One leg low PD controller gains
    p_gains(0, 0) = 450.0 * 0.03;
    p_gains(1, 1) = 450.0 * 0.03;
    d_gains(0, 0) = 2 * sqrt(p_gains(0, 0));
    d_gains(1, 1) = 2 * sqrt(p_gains(1, 1));

    // inital guesses from calculations - with foot plates
    // inital_guess << -0.658, 0.821, 0.830, 0, 1.69, 0, 0, 5, 0, 3.26;
    // The below is the best guess AFFC has achieved over 222 steps 
    // inital_guess << -0.0155, -0.58,	0.47, 0.286, -0.679, 0.0674, 0.802, 5.00, -1.19, 2.95;

    // setup AFFC controller and set Criterion 1 and Criterion 2
    affc = new AdaptiveController<double, 2, 50>(lengths, learning_rate, p_gains, d_gains);
    affc->set_criterions(deg2rad(2), deg2rad(0.2));
    affc->set_inital_guess(inital_guess);

    // unknown parameter logger
    qref_logger = spdlog::basic_logger_mt("qref_logger", "logs/affc_qref_traj.log", true);
    qref_dot_logger = spdlog::basic_logger_mt("qref_dot_logger", "logs/affc_qref_dot_traj.log", true);
    qref_ddot_logger = spdlog::basic_logger_mt("qref_ddot_logger", "logs/affc_qref_ddot_traj.log", true);
    lambda_logger = spdlog::basic_logger_mt("lambda_logger", "logs/affc_lambdas.log", true);
    tracking_error_logger = spdlog::basic_logger_mt("tracking_error_logger", "logs/affc_tracking_error.log", true);
    complete_logger = spdlog::basic_logger_mt("complete_logger", "logs/affc_complete.log", true);
    torque_logger = spdlog::basic_logger_mt("torque_logger", "logs/affc_torques.log", true);
    qact_logger = spdlog::basic_logger_mt("qact_logger", "logs/affc_qact.log", true);
    qerr_logger = spdlog::basic_logger_mt("qerr_logger", "logs/pd_err.log", true);
    affc_grad_logger = spdlog::basic_logger_mt("affc_grad_logger", "logs/affc_grad.log", true);
    spdlog::flush_every(std::chrono::seconds(2));
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

    if(controller_mode_ == 0) {
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // desiredJointPositions_ = posReader_.getNextPos();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        desiredJointPositions_[0] = 0.3*sin(2.0*M_PI/5*time);
        desiredJointPositions_[1] = 0.2*sin(2.0*M_PI/5*time) - 0.5;

        pdController.loop(desiredJointPositions_, robot_->getPosition());
        desiredJointTorques_ = pdController.output();

        // qact_logger->info("{},{},{},{}", robot_->getPosition()[0], robot_->getPosition()[1], robot_->getPosition()[2], robot_->getPosition()[3]);
        // qerr_logger->info("{},{},{},{}", pdController.get_err_prev()[0], pdController.get_err_prev()[1], pdController.get_err_prev()[2], pdController.get_err_prev()[3]);

        complete_logger->info("{},{},{},{}", desiredJointPositions_[0], desiredJointPositions_[1], desiredJointTorques_[0], desiredJointTorques_[1]);

        torque_limiter(80.0);

        robot_->setTorque(desiredJointTorques_);
    }
    else if(controller_mode_ == 0){
        // AFFC Controller Mode

        // switch motor control mode to torque control
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // we only want to loop through one leg at a time 
        // desiredJointPositions_ = posReader_.getNextPos();
        double time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time0).count()/1.0e9;
        desiredJointPositions_[0] = 0.2*sin(2.0*M_PI/5*time);
        desiredJointPositions_[1] = 0.1*sin(2.0*M_PI/5*time) - 0.5;
        desiredJointVelocities_[0] = (0.2*2.0*M_PI/5)*cos(2.0*M_PI/5*time);
        desiredJointVelocities_[1] = (0.1*2.0*M_PI/5)*cos(2.0*M_PI/5*time);
        desiredJointAccelerations_[0] = -0.2*pow(2.0*M_PI/5, 2)*sin(2.0*M_PI/5*time);
        desiredJointAccelerations_[1] = -0.1*pow(2.0*M_PI/5, 2)*sin(2.0*M_PI/5*time);
        for(int j = 0; j < X2_NUM_JOINTS / 2; j++) {
            
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
        // vel_limiter(deg2rad(rateLimit));

        // check if the AFFC algorithm is finished
        if (affc->is_finished()) {
            spdlog::info("AFFC is finished");
            return;
        }

        // current AFFC only works with one leg at a time
        Eigen::Matrix<double, 2, 1> refPos, refVel, refAccel;
        Eigen::Matrix<double, 2, 1> actualPos;

        refPos << desiredJointPositions_[0], desiredJointPositions_[1];
        refVel << desiredJointVelocities_[0], desiredJointVelocities_[1];
        refAccel << desiredJointAccelerations_[0], desiredJointAccelerations_[1];

        // filter qact to remove noise
        // lowPass.filter(robot_->getPosition());
        // actualPos << lowPass.output()[0], lowPass.output()[1];
        actualPos << robot_->getPosition()[0], robot_->getPosition()[1];

        // iterate the AFFC algorithm once
        if (period_counter_ * 5.0 < time) {
            period_counter_++;
            spdlog::info("AFFC cycle {} complete", period_counter_);
            // a complete cycle of the trajectory has been completed
            // affc->loop(refPos, actualPos, true);
            affc->loop(refPos, actualPos, refVel, refAccel, true);

            // log unknown parameters and tracking error over iterations
            Eigen::Matrix<double, 16, 1> lambdas = affc->peek_learned_params();
            Eigen::Matrix<double, 2, 1> tracking_err = affc->peek_tracking_error();
            lambda_logger->info("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}", 
                                lambdas(0, 0),
                                lambdas(1, 0),
                                lambdas(2, 0),
                                lambdas(3, 0),
                                lambdas(4, 0),
                                lambdas(5, 0),
                                lambdas(6, 0),
                                lambdas(7, 0),
                                lambdas(8, 0),
                                lambdas(9, 0),
                                lambdas(10, 0),
                                lambdas(11, 0),
                                lambdas(12, 0),
                                lambdas(13, 0),
                                lambdas(14, 0),
                                lambdas(15, 0)
            );
            tracking_error_logger->info("{}, {}", tracking_err(0, 0), tracking_err(1, 0));
            spdlog::info("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}", 
                                lambdas(0, 0),
                                lambdas(1, 0),
                                lambdas(2, 0),
                                lambdas(3, 0),
                                lambdas(4, 0),
                                lambdas(5, 0),
                                lambdas(6, 0),
                                lambdas(7, 0),
                                lambdas(8, 0),
                                lambdas(9, 0),
                                lambdas(10, 0),
                                lambdas(11, 0),
                                lambdas(12, 0),
                                lambdas(13, 0),
                                lambdas(14, 0),
                                lambdas(15, 0)
            );
            spdlog::info("{}, {}", tracking_err(0, 0), tracking_err(1, 0));

            // update the number of trajectory cycles that have been completed
            completed_cycles_ = posReader_.getCycles();
        } else {
            // affc->loop(refPos, actualPos, false);
            affc->loop(refPos, actualPos, refVel, refAccel, false);
        }

        // affc gradient descent of the tracking error function
        affcFbTorque = affc->peek_grad_descent_error(); 
        affc_grad_logger->info("{}, {}", affcFbTorque(0, 0), affcFbTorque(1, 0));

        // obtain required joint torques from AFFC control loop to reach desiredJointPositions_
        Eigen::Matrix<double, 2, 1> affc_out = affc->output();
        desiredJointTorques_ << affc_out(0, 0), affc_out(1, 0), 0, 0;

        complete_logger->info("{},{},{},{},{},{}", posReader_.getGaitIndex(), refPos[0], refPos[1], desiredJointTorques_[0], desiredJointTorques_[1], time);
        qact_logger->info("{}, {}", robot_->getPosition()[0], robot_->getPosition()[1]);

        // limit torques
        torque_limiter(80.0);

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

void X2DemoState::torque_limiter(const double limit) {
    for (std::size_t i = 0; i < X2_NUM_JOINTS; i++) {
        if (abs(desiredJointTorques_[i]) > limit) {
            if (desiredJointTorques_[i] > 0) {
                desiredJointTorques_[i] = limit;
            } else {
                desiredJointTorques_[i] = -limit;
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