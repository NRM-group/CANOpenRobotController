/**
 * @file IdleState.cpp
 * @author Josh Lai 
 * @brief 
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "IdleState.h"

void IdleState::entry(void) 
{
    spdlog::info("Idle/read state");
    std::cout << "Idle State Entered, joint angles can be read"  << std::endl;
    std::cout << "===================" << std::endl;
    std::cout << "Move Stand \"W\"" << std::endl;
    std::cout << "Move Sit \"S\"" << std::endl;
    std::cout << "===================" << std::endl;
    m_robot->initTorqueControl();
}

void IdleState::during(void) 
{
    std::cout << "Joint 1: " << m_robot->getPosition()[0] << std::endl;
    std::cout << "Joint 2: " << m_robot->getPosition()[1] << std::endl;
    std::cout << "Joint 3: " << m_robot->getPosition()[2] << std::endl;
    std::cout << "Joint 4: " << m_robot->getPosition()[3] << std::endl;
}

void IdleState::exit(void) 
{
    m_robot->initPositionControl();
    m_robot->setPosControlContinuousProfile(true);
    spdlog::info("Idle State Exited");
}