#include "ExoState.hpp"
#define LOG(x)  spdlog::info("[OffState]: {}", x)
#define ERR(x)  spdlog::error("[OffState]: {}", x)

OffState::OffState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Off State"), _Robot(robot), _Node(node),
    _Counter{}
{
}

void OffState::entry()
{
    LOG(">>> Entered >>>");
    _Robot->disable();
}

void OffState::during()
{
    if (_Counter++ % 1000 == 0) {
        if (!_Node->ok()) {
            spdlog::warn("[OffState]: Node status {}", _Node->get_heart_beat().status);
        }
        else {
            spdlog::info("[OffState]: Robot idling in disabled state");
        }
    }
}

void OffState::exit()
{
    LOG("<<< Exited <<<");
}