#include "ExoState.hpp"
#define LOG(x)  spdlog::info("[OffState]: {}", x)

OffState::OffState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Off State"), _Robot(robot), _Node(node), _Counter()
{
}

void OffState::entry()
{
    _Robot->initTorqueControl();
    usleep(100);
    _Robot->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
    LOG(">>> Entered >>>");
}

void OffState::during()
{
    if (!(_Counter++ % (333 * 5))) {
        if (!_Node->ok()) {
            spdlog::warn("[OffState]: Node status {}", _Node->get_heart_beat().status);
        }
        else { // TODO:
            spdlog::info("You found the Easter egg :)");
        }
    }
}

void OffState::exit()
{
    LOG("<<< Exited <<<");
}