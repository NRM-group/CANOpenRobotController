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

    // butterworth 2nd order fc = 15Hz, fs = 333.333Hz
    // lowPass.set_coeff_a({1.0, -0.7478, 0.2722});
    // lowPass.set_coeff_b({0.1311, 0.2622, 0.1311});
    // butterworth 2nd order fc = 30Hz, fs = 333.333Hz
    lowPass.set_coeff_a({1.0, -1.2247, 0.4504});
    lowPass.set_coeff_b({0.0564, 0.1129, 0.0564});

    Eigen::Matrix<double, 2, 2> p_gains = Eigen::Matrix<double, 2, 2>::Zero();
    Eigen::Matrix<double, 2, 2> d_gains = Eigen::Matrix<double, 2, 2>::Zero();
    Eigen::Matrix<double, 2, 1> lengths = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 18, 1> learning_rate = Eigen::Matrix<double, 18, 1>::Zero(); 
    Eigen::Matrix<double, 18, 1> inital_guess = Eigen::Matrix<double, 18, 1>::Zero();

    // single length hip and shank lengths - without footplates
    lengths << 0.4, 0.36;

    // AFFC learning rates
    learning_rate << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    learning_rate *= 8e-3;

    // One leg low PD controller gains
    p_gains(0, 0) = 450.0 * 0.09;
    p_gains(1, 1) = 450.0 * 0.09;
    d_gains(0, 0) = 2 * sqrt(p_gains(0, 0));
    d_gains(1, 1) = 2 * sqrt(p_gains(1, 1));

    // inital guess with best tracking - different values for different gait speeds???
    inital_guess << 8.2705, 0.6652, -1.2842, -2.1374, 2.7300, 5.2709, 4.6353, -11.9225, -1.0878, -0.5561, 13.0375, 11.5076, 5.9097, -4.4126, 9.1166, 14.2331, 2.1372, 2.1077;

    // setup AFFC controller and set Criterion 1 and Criterion 2
    affc = new AdaptiveController<double, 2, 50>(lengths, learning_rate, p_gains, d_gains);
    affc->set_criterions(deg2rad(0.7), deg2rad(0.2));
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
    affc_lambda_debug_logger = spdlog::basic_logger_mt("affc_lambda_debug_logger", "logs/affc_lambda_debug.log", true);
    spdlog::flush_every(std::chrono::seconds(1));
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
    if(controller_mode_ == 1){
        // AFFC Tune Mode

        // switch motor control mode to torque control
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // we only want to loop through one leg at a time 
        double time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time0).count()/1.0e9;

        desiredJointPositions_ = gaitTrajectory_.getPosition(time);
        desiredJointVelocities_ = gaitTrajectory_.getVelocity(time);
        desiredJointAccelerations_ = gaitTrajectory_.getAccelaration(time);
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
        lowPass.filter(robot_->getPosition());
        actualPos << lowPass.output()[0], lowPass.output()[1];
        // actualPos << robot_->getPosition()[0], robot_->getPosition()[1];

        // iterate the AFFC algorithm once
        if (period_counter_ * 5.0 < time) {
            period_counter_++;
            spdlog::info("AFFC cycle {} complete", period_counter_);
            // a complete cycle of the trajectory has been completed
            affc->loop(refPos, actualPos, refVel, refAccel, true);

            // log unknown parameters and tracking error over iterations
            Eigen::Matrix<double, 18, 1> lambdas = affc->peek_learned_params();
            Eigen::Matrix<double, 2, 1> tracking_err = affc->peek_tracking_error();
            lambda_logger->info("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}", 
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
                                lambdas(15, 0),
                                lambdas(16, 0),
                                lambdas(17, 0)
            );
            tracking_error_logger->info("{}, {}", tracking_err(0, 0), tracking_err(1, 0));
            spdlog::info("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}", 
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
                                lambdas(15, 0),
                                lambdas(16, 0),
                                lambdas(17, 0)
            );
            spdlog::info("{}, {}", tracking_err(0, 0), tracking_err(1, 0));
        } else {
            // affc->loop(refPos, actualPos, false);
            affc->loop(refPos, actualPos, refVel, refAccel, false);
        }

        // log lambdas at each loop - for debug purposes
        auto lambdas = affc->peek_current_learned_params();

        // affc gradient descent of the tracking error function
        affcFbTorque = affc->peek_grad_descent_error(); 
        affc_grad_logger->info("{}, {}", affcFbTorque(0, 0), affcFbTorque(1, 0));

        // obtain required joint torques from AFFC control loop to reach desiredJointPositions_
        Eigen::Matrix<double, 2, 1> affc_out = affc->output();
        desiredJointTorques_ << affc_out(0, 0), affc_out(1, 0), 0, 0;

        complete_logger->info("{},{},{},{},{}", refPos[0], refPos[1], desiredJointTorques_[0], desiredJointTorques_[1], time);
        qact_logger->info("{}, {}", robot_->getPosition()[0], robot_->getPosition()[1]);

        // limit torques
        torque_limiter(80.0);

        // update motor torques to required values 
        robot_->setTorque(desiredJointTorques_);
    } else if (controller_mode_ == 0) {
        // AFFC Feedforward Mode

        // switch motor control mode to torque control
        if (robot_->getControlMode() != CM_TORQUE_CONTROL) {
            robot_->initTorqueControl();
            spdlog::info("Initalised Torque Control Mode");
        }

        // we only want to loop through one leg at a time 
        double time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time0).count()/1.0e9;

        desiredJointPositions_ = gaitTrajectory_.getPosition(time);
        desiredJointVelocities_ = gaitTrajectory_.getVelocity(time);
        desiredJointAccelerations_ = gaitTrajectory_.getAccelaration(time);
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

        // current AFFC only works with one leg at a time
        Eigen::Matrix<double, 2, 1> refPos, refVel, refAccel;
        Eigen::Matrix<double, 2, 1> actualPos;

        refPos << desiredJointPositions_[0], desiredJointPositions_[1];
        refVel << desiredJointVelocities_[0], desiredJointVelocities_[1];
        refAccel << desiredJointAccelerations_[0], desiredJointAccelerations_[1];

        // filter qact to remove noise
        lowPass.filter(robot_->getPosition());
        actualPos << lowPass.output()[0], lowPass.output()[1];

        affc->ff_loop(refPos, actualPos, refVel, refAccel);
        Eigen::Matrix<double, 2, 1> affc_out = affc->output();
        desiredJointTorques_ << affc_out(0, 0), affc_out(1, 0), 0, 0;

        torque_limiter(80);

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