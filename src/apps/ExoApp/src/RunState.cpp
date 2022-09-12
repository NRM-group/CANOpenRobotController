#include "ExoState.hpp"

RunState::RunState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot)
    : State(state_machine), _Robot(robot)
{
    spdlog::info("RunState: Ready");
}

RunState::~RunState()
{
    spdlog::info("RunState: Destructed");
}

void RunState::entry()
{
    _Robot->initTorqueControl();
}

void RunState::during()
{

}

void RunState::exit()
{
    spdlog::info("RunState: Call to exit()");
}