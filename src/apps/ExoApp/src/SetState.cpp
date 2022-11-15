#include "ExoState.hpp"
#define LOG(x) spdlog::info("[SetState]: {}", x)
#define ERR(x) spdlog::error("[SetState]: {}", x)

SetState::SetState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Set State"), _Robot(robot), _Node(node)
{
    ;
}

void SetState::entry()
{
    ;
}

void SetState::during()
{
    ;
}

void SetState::exit()
{
    LOG("<<< Exited <<<");
}