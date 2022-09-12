#include "ExoState.hpp"

SetState::SetState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot)
    : State(state_machine), _Robot(robot)
{
    spdlog::info("SetState: Ready");
}

SetState::~SetState()
{
    spdlog::info("SetState: Destructed");
}

void SetState::entry()
{
    _Robot->initTorqueControl();
}

void SetState::during()
{

}

void SetState::exit()
{
    spdlog::info("SetState: Call to exit()");
}