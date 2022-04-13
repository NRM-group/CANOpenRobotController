#include "InitState.h"

void
InitState::entry(void)
{
    spdlog::info("InitState Entered");
    std::cout << "Press D to transition the idle state" << std::endl;
    std::cout << "Press S to transition to sitting state" << std::endl;
    std::cout << "Press W to transition to stand state" << std::endl;
    std::cout << "Press A to calibrate the Exo State" << std::endl;
    m_robot->resetErrors();
    std::cout << "Reset Complete" << std::endl;
}

void InitState::during(void) { }

void InitState::exit(void)
{
    m_robot->initPositionControl();
    m_robot->setPosControlContinuousProfile(true);
    spdlog::info("InitState exited");
}
