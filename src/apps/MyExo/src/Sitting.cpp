/**
 * @file Sitting.cpp
 * @author 
 * @brief 
 * @version 0.1
 * @date 2022-03-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "Sitting.h"

void Sitting::entry(void) 
{
    spdlog::info("Sitting State entered");
    std::cout << "SITTING DOWN" << std::endl;

    //Initialise the Trajectory Generator
    m_trajectory->initialise_trajectory(SIT, 2, m_robot->getPosition());
    currTrajProgress = 0;
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
}

void Sitting::during(void) 
{
    timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);

    double elapsedSec = currTime.tv_sec - prevTime.tv_sec + (currTime.tv_nsec - prevTime.tv_nsec) / 1e9;
    prevTime = currTime;

    currTrajProgress += elapsedSec;
    m_robot->setPosition(m_trajectory->get_set_point(currTrajProgress));
}

void Sitting::exit(void) 
{
    spdlog::info("Sitting Down Stat Exited");
}