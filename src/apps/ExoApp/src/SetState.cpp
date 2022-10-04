#include "ExoState.hpp"
#define LOG(x)  spdlog::info("[SetState]: {}", x)

SetState::SetState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Set State"), _Robot(robot), _Node(node)
{
}

void SetState::entry()
{
    _Robot->initTorqueControl();

    LOG(">>> Entered >>>");

    _Robot->calibrateForceSensors();
    // TODO:
    // std::string asdf;
    // _Node->config_path(asdf);
    // YAML::Node temp = YAML::LoadFile(asdf);
    // std::ofstream os(asdf);
    // os << temp;
}

void SetState::during()
{
}

void SetState::exit()
{
    LOG("<<< Exited <<<");
}