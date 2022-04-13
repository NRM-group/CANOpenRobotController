#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#include <array>
#include <Eigen/Dense>

#include "LogHelper.h"
#include "X2Robot.h"

// Comment to use PID
#define _OPTIMISE_PD

class Controller
{
public:
    /**
     * \brief Default constructor.
     */
    Controller(void);

    /**
     * \brief Initialise PID controller with zero gains.
     * \param joint_count Number of joints (expects 4).
     * \param dt The time interval between each loop.
     * \return 0 on success and 1 if failed to initialise.
     */
    int init(int joint_count, double dt);

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
    void set_proportional(double kp);

    /**
     * \brief Sets the derivative gain.
     */
    void set_derivative(double kd);

    /**
     * \brief Sets the integral gain.
     */
    void set_integral(double ki);

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
    double get_proportional(void);

    /**
     * \brief Returns the derivative gain.
     */
    double get_derivative(void);

    /**
     * \brief Returns the integral gain.
     */
    double get_integral(void);

    /**
     * \brief Calculates the control loop of the PID (PD) controller.
     * \param position The current encoded position.
     * \param desired The desired positions of each joint.
     * \return The torque output of the controller.
     */
    Eigen::VectorXd control_loop(
        Eigen::VectorXd position, Eigen::VectorXd desired
    );

private:
    /* Robot */
    int joint_count;
    double dt;

    /* Gains */
    double Kp;
    double Kd;
    double Ki;

    /* Error */
    Eigen::VectorXd error_prev;
    Eigen::VectorXd error_sum;
};

#endif//__CONTROLLER_HPP
