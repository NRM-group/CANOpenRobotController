
#include "ExoState.hpp"

OffState::OffState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot)
    : State(state_machine), _Robot(robot)
{
    spdlog::info("OffState: Ready");
}

OffState::~OffState()
{
    spdlog::info("OffState: Destructed");
}

void OffState::entry()
{
    _Robot->initTorqueControl();
    _Robot->set_torque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
    spdlog::info("OffState: Entry");
}

void OffState::during() // TODO:
{
    static std::size_t i = 0;
    if (!(i++ % 333))
    {
        spdlog::info("You found the Easter egg :)");
    }
}

void OffState::exit()
{
    spdlog::info("OffState: Exit");
}