#include "controller.hpp"

Controller::Controller() { }

int
Controller::init(int joint_count, double dt)
{
    if (joint_count == 4) {
        this->joint_count = joint_count;
    } else {
        spdlog::error("Expected joint count 4, but got {}", joint_count);
        spdlog::error("Controller not initialised");
        return 1;
    }
    this->dt = dt;
    this->Kp = 0;
    this->Kd = 0;
    this->Ki = 0;
    this->error_prev = Eigen::VectorXd::Zero(joint_count);
    this->error_sum = Eigen::VectorXd::Zero(joint_count);
    spdlog::info("Controller initialised");
    return 0;
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

inline void
Controller::set_proportional(double kp)
{
    Kp = kp;
}

inline void
Controller::set_derivative(double kd)
{
    Kd = kd;
}

inline void
Controller::set_integral(double ki)
{
    Ki = ki;
}

inline std::array<double, 3>
Controller::get_pid_gains(void)
{
    return std::array<double, 3>{
        Kp, Kd, Ki
    };
}

inline std::array<double, 2>
Controller::get_pd_gains(void)
{
    return std::array<double, 2>{
        Kp, Kd
    };
}

inline double
Controller::get_proportional(void)
{
    return Kp;
}

inline double
Controller::get_derivative(void)
{
    return Kd;
}

inline double
Controller::get_integral(void)
{
    return Ki;
}

Eigen::VectorXd
Controller::control_loop(Eigen::VectorXd position, Eigen::VectorXd desired)
{
    Eigen::VectorXd output(joint_count);
    for (int i = 0; i < joint_count; i++) {
        double pos = position[i];
        // Proportional
        double error = desired[i] - pos;
        output[i] += Kp * error;
        // Derivative
        double errorSlope = (error - error_prev[i]) / dt;
        output[i] += Kd * errorSlope;
        // Integral
#ifndef _OPTIMISE_PD
        error_sum[i] += error * dt;
        output[i] += Ki * error_sum(i);
#endif
        error_prev[i] = error;
    }
    return output;
}
