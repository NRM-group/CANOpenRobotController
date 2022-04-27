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
#include "pd_controller.hpp"
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
    X2DemoState(StateMachine *m, X2Robot *exo, const char *name = NULL);

    Eigen::VectorXd& getDesiredJointTorques();
    Eigen::VectorXd& getDesiredJointVelocities();


    Eigen::VectorXd enableJoints;

    double kp, kd;
    double debug_torque;


private:
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig> server_;
    void dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level);

    double t_step_ = 0.003; // 0.003 todo: get from main
    int freq_ = 1 / t_step_;
    int t_count_ = 0;
    int state_ = STEP_DOWN;

    PDController<double> pd;
    int controller_mode_;

    std::chrono::steady_clock::time_point time0;
    Eigen::VectorXd desiredJointTorques_;
    Eigen::VectorXd desiredJointVelocities_;

    Eigen::VectorXd kTransperancy_;
    double amplitude_, period_, offset_;

};

#endif