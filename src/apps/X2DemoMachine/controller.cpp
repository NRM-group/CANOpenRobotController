#include "controller.hpp"

Controller::Controller(void) {
    ;
}

Controller::Controller(int joint_count, double dt) : dt(dt)
{
    if (joint_count == 4) {
        this->joint_count = joint_count;
    } else {
        spdlog::error("Expected joint count 4, but got %d", joint_count);
        spdlog::error("Controller not initialised");
        return;
    }
    Kp = 0;
    Kd = 0;
    Ki = 0;
    error_prev = Eigen::VectorXd(joint_count);
    error_sum = Eigen::VectorXd(joint_count);
    spdlog::info("Controller initialised");
}

void
Controller::init(int joint_count, double dt)
{
    if (joint_count == 4) {
        this->joint_count = joint_count;
    } else {
        spdlog::error("Expected joint count 4, but got %d", joint_count);
        spdlog::error("Controller not initialised");
        return;
    }
    this->dt = dt;
}

void
Controller::set_pid_gains(double kp, double kd, double ki)
{
    set_proportional(kp);
    set_derivative(kd);
    set_integral(ki);
}

void
Controller::set_pd_gains(double kp, double kd)
{
    set_proportional(kp);
    set_derivative(kd);
}

void
Controller::set_proportional(double kp)
{
    Kp = kp;
    spdlog::info("Updated proportional gain: %f", Kp);
}

void
Controller::set_derivative(double kd)
{
    Kd = kd;
    spdlog::info("Updated derivative gain: %f", Kd);
}

void
Controller::set_integral(double ki)
{
    Ki = ki;
    spdlog::info("Updated integral gain: %f", Ki);
}

std::array<double, 3>
Controller::get_pid_gains(void)
{
    return std::array<double, 3>{
        Kp, Kd, Ki
    };
}

std::array<double, 2>
Controller::get_pd_gains(void)
{
    return std::array<double, 2>{
        Kp, Kd
    };
}

double
Controller::get_proportional(void)
{
    return Kp;
}

double
Controller::get_derivative(void)
{
    return Kd;
}

double
Controller::get_integral(void)
{
    return Ki;
}

Eigen::VectorXd
Controller::control_loop(Eigen::VectorXd position, Eigen::VectorXd desired)
{
    Eigen::VectorXd output(joint_count);
    for (int i = 0; i < joint_count; i++) {
        double pos = position(i);
        // Proportional
        double error = desired(i) - pos;
        output(i) += Kp * error;
        // Derivative
        double errorSlope = (error - error_prev(i)) / dt;
        output(i) += Kd * errorSlope;
        // Integral
#ifndef _OPTIMISE_PD
        error_sum(i) += error * dt;
        output(i) += Ki * error_sum(i);
#endif
        error_prev(i) = error;
    }
    return output;
}
