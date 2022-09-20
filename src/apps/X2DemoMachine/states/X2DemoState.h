/**
 * /file X2DemoState.h
 * /author Emek Baris Kucuktabak
 * /brief Concrete implementation of DemoState
 * /version 1.1
 * /date 2022-02-22
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef SRC_X2DEMOSTATE_H
#define SRC_X2DEMOSTATE_H

#include "State.h"
#include "X2Robot.h"
#include "controller.hpp"
#include "LookupTable.hpp"
#include <ctime>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <math.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <CORC/dynamic_paramsConfig.h>

#define STEP_UP         1
#define STEP_DOWN       0

#define LIMIT_TORQUE    80 // [Nm]

#define LEFT_HIP        0
#define LEFT_KNEE       1
#define RIGHT_HIP       2
#define RIGHT_KNEE      3

using namespace ctrl;

/**
 * \brief Demo State for the X2DemoMachine
 *
 *
 */
class X2DemoState : public State {
    X2Robot *robot_;

public:
    void entry(void);
    void during(void);
    void exit(void);
    X2DemoState(StateMachine *m, X2Robot *exo, const float updateT, const char *name = NULL);
    ~X2DemoState(void);

    Eigen::VectorXd& getDesiredJointTorques();
    Eigen::VectorXd& getDesiredJointPositions();
    Eigen::VectorXd& getDesiredJointTorquesPSplit();
    Eigen::VectorXd& getDesiredJointTorquesISplit();
    Eigen::VectorXd& getDesiredJointTorquesDSplit();
    Eigen::VectorXd& getDesiredJointVelocities();

    Eigen::VectorXd enableJoints;

    int controller_mode_;
    double maxTorqueLimit;
    double rateLimit;
    double refPos1;
    double refPos2;
    int refPosPeriod;
    AdaptiveController<double, 2, 50>* affc;
    std::shared_ptr<spdlog::logger> qref_logger; 
    std::shared_ptr<spdlog::logger> qref_dot_logger; 
    std::shared_ptr<spdlog::logger> qref_ddot_logger; 
    std::shared_ptr<spdlog::logger> lambda_logger;
    std::shared_ptr<spdlog::logger> tracking_error_logger;
    std::shared_ptr<spdlog::logger> complete_logger;
    std::shared_ptr<spdlog::logger> torque_logger;
    std::shared_ptr<spdlog::logger> qact_logger;
    std::shared_ptr<spdlog::logger> qerr_logger;
    std::shared_ptr<spdlog::logger> affc_grad_logger;
    Eigen::VectorXd debugTorques;
    Eigen::VectorXd frictionCompensationTorques;
    Eigen::MatrixXd affcFbTorque;

    PDController<double, X2_NUM_JOINTS> pdController;
    Butterworth<double, X2_NUM_JOINTS, 2> lowPass;

private:
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig> server_;
    void dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level);
    void vel_limiter(const double limit);
    void torque_limiter(const double limit);
    void addDebugTorques(int joint);
    void addFrictionCompensationTorques(int joint);

    const int freq_;
    int t_count_ = 0;
    int state_ = STEP_DOWN;

    std::chrono::steady_clock::time_point time0;
    Eigen::VectorXd desiredJointPositions_;         // the desired joint positions
    Eigen::VectorXd prevDesiredJointPositions_;     // the previous desired joint position set by rate limiter
    Eigen::VectorXd desiredJointVelocities_;
    Eigen::VectorXd desiredJointAccelerations_;
    Eigen::VectorXd desiredJointTorques_;
    Eigen::VectorXd desiredJointTorquesP_;
    Eigen::VectorXd desiredJointTorquesI_;
    Eigen::VectorXd desiredJointTorquesD_;

    Eigen::VectorXd kTransperancy_;
    double amplitude_, period_, offset_;
    double period_counter_;

    LookupTable<double, X2_NUM_JOINTS> posReader_;
    LookupTable<double, X2_NUM_JOINTS> velReader_;
    LookupTable<double, X2_NUM_JOINTS> accelReader_;
    int completed_cycles_;
};

#endif
