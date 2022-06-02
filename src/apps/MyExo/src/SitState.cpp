#include "SitState.h"

void
SitState::entry(void)
{
    spdlog::info("Sit State Entered");
    std::cout << "Press W to stand" << std::endl;
    std::cout << "Press D to return to the Idle State" << std::endl;
}

void SitState::during(void) { }

void SitState::exit(void) 
{
    m_robot->initPositionControl();
    m_robot->setPosControlContinuousProfile(true);
    spdlog::info("SitState exited");
}
