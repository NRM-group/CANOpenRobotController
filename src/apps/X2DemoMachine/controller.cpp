#include "controller.h"

void
ctrl::init(X2Robot* robot, int joint_count, double dt)
{
    if (joint_count == 4) {
        ctrl::joint_count = joint_count;
    } else {
        spdlog::error("Expected joint count 4, but got %d", joint_count);
        spdlog::error("Controller not initialised");
        return;
    }
    ctrl::robot = robot;
    ctrl::dt = dt;
    ctrl::Kp = 0;
    ctrl::Kd = 0;
    ctrl::Ki = 0;
    ctrl::error_prev = Eigen::VectorXd(ctrl::joint_count);
    ctrl::error_sum = Eigen::VectorXd(ctrl::joint_count);
    spdlog::info("Controller initialised");
}

void
ctrl::set_pid_gains(double kp, double kd, double ki)
{
    ctrl::set_proportional(kp);
    ctrl::set_derivative(kd);
    ctrl::set_integral(ki);
}

void
ctrl::set_pd_gains(double kp, double kd)
{
    ctrl::set_proportional(kp);
    ctrl::set_derivative(kd);
}

void
ctrl::set_proportional(double kp)
{
    ctrl::Kp = kp;
    spdlog::info("Updated proportional gain: %f", ctrl::Kp);
}

void
ctrl::set_derivative(double kd)
{
    ctrl::Kd = kd;
    spdlog::info("Updated derivative gain: %f", ctrl::Kd);
}

void
ctrl::set_integral(double ki)
{
    ctrl::Ki = ki;
    spdlog::info("Updated integral gain: %f", ctrl::Ki);
}

std::array<double, 3>
ctrl::get_pid_gains(void)
{
    return std::array<double, 3>{
        ctrl::Kp, ctrl::Kd, ctrl::Ki
    };
}

std::array<double, 2>
ctrl::get_pd_gains(void)
{
    return std::array<double, 2>{
        ctrl::Kp, ctrl::Kd
    };
}

double
ctrl::get_proportional(void)
{
    return ctrl::Kp;
}

double
ctrl::get_derivative(void)
{
    return ctrl::Kd;
}

double
ctrl::get_integral(void)
{
    return ctrl::Ki;
}

Eigen::VectorXd
ctrl::control_loop(Eigen::VectorXd desired)
{
    Eigen::VectorXd output(ctrl::joint_count);
    for (int i = 0; i < ctrl::joint_count; i++) {
        double pos = ctrl::robot->getPosition()(i);
        // Proportional
        double error = desired(i) - pos;
        output(i) += ctrl::Kp * error;
        // Derivative
        double errorSlope = (error - ctrl::error_prev(i)) / ctrl::dt;
        output(i) += ctrl::Kd * errorSlope;
        // Integral
#ifndef _OPTIMISE_PD
        ctrl::error_sum(i) += error * ctrl::dt;
        output(i) += ctrl::Ki * ctrl::error_sum(i);
#endif
        ctrl::error_prev(i) = error;
    }
    return output;
}

