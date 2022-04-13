/**
 * @file ControlState.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ControlState.h"

void ControlState::entry(void)
{
    spdlog::info("ControlState Entered");
    std::cout << "Press W to Stand" << std::endl;

    ctrl::init(m_robot, m_robot->motorDrives.size(), 1/1000);
}

void ControlState::during(void)
{
    Eigen::VectorXd desired(0, 0, 0, 0);
    Eigen::VectorXd torques = ctrl::control_loop(desired);
}

void ControlState::exit(void)
{
    spdlog::info("ControlState exited");
}