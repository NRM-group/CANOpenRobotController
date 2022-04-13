#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <array>
#include <Eigen/Dense>

#include "LogHelper.h"
#include "X2Robot.h"

// Comment to use PID
#define _OPTIMISE_PD

namespace ctrl
{
/* Robot */
static int joint_count;
static X2Robot* robot;
static double dt;

/* Gains */
static double Kp;
static double Kd;
static double Ki;

/* Error */
static Eigen::VectorXd error_prev;
static Eigen::VectorXd error_sum;

/**
 * \brief Initialise PID controller with zero gains.
 * \param x2Robot A pointer to the robot object.
 * \param jointCount Number of joints (expects 4).
 * \param dt The time interval between each loop.
 */
void init(X2Robot* robot, int joint_count, double dt);

/**
 * \brief Sets the gains proportional, derivative, and integral.
 */
void set_pid_gains(double kp, double kd, double ki);

/**
 * \brief Sets the gains proportional and derivative.
 */
void set_pd_gains(double kp, double kd);

/**
 * \brief Sets the proportional gain.
 */
inline void set_proportional(double kp);

/**
 * \brief Sets the derivative gain.
 */
inline void set_derivative(double kd);

/**
 * \brief Sets the integral gain.
 */
inline void set_integral(double ki);

/**
 * \brief Returns the PID gains.
 */
std::array<double, 3> get_pid_gains(void);

/**
 * \brief Returns the PD gains.
 */
std::array<double, 2> get_pd_gains(void);

/**
 * \brief Returns the proportional gain.
 */
inline double get_proportional(void);

/**
 * \brief Returns the derivative gain.
 */
inline double get_derivative(void);

/**
 * \brief Returns the integral gain.
 */
inline double get_integral(void);

/**
 * \brief Calculates the control loop of the PID (PD) controller.
 * \param desired The desired positions of each joint.
 * \return The torque required in each joint.
 */
Eigen::VectorXd control_loop(
    Eigen::VectorXd desired=Eigen::VectorXd(ctrl::joint_count)
);
}

#endif//__CONTROLLER_H
