#include "Standing.h"

void Standing::entry(void) 
{
    spdlog::info("Standing Up State Entered");
    std::cout << "=============================" << std::endl
              << "Press S to return to sitting" << std::endl
              << "Press D to move to IDLE state" << std::endl
              << "=============================" << std::endl;
    m_trajectory->initialise_trajectory(STAND, 2, m_robot->getPosition());
    currTrajProgress = 0;
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
}

void Standing::during(void)
{
    timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);

    double elapsedSec = currTime.tv_sec - prevTime.tv_sec + (currTime.tv_nsec - prevTime.tv_nsec) / 1e9;
    prevTime = currTime;
    currTrajProgress += elapsedSec;
    m_robot->setPosition(m_trajectory->get_set_point(currTrajProgress));

}

void Standing::exit(void)
{
    spdlog::info("Standing up State Exited");
}
