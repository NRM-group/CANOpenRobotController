#include "ExoState.hpp"
#define LOG(x)  spdlog::info("[OffState]: {}", x)
#define ERR(x)  spdlog::error("[OffState]: {}", x)

OffState::OffState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Off State"), _Robot(robot), _Node(node),
    _Counter{}
{
    ;
}

void OffState::entry()
{
    _Robot->disable();
    LOG(">>> Entered >>>");
}

void OffState::during()
{
    ;
}

void OffState::exit()
{
    LOG("<<< Exited <<<");
}