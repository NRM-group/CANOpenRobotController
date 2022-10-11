#include "ExoState.hpp"
#define LOG(x)      spdlog::info("[SetState]: {}", x)
#define ERR(x)      spdlog::error("[SetState]: {}", x)
#define PAUSE       usleep(1000)
#define RANGE       16
#define SAMPLES     50

SetState::SetState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Set State"), _Robot(robot), _Node(node)
{
}

void SetState::entry()
{
    LOG(">>> Entered >>>");
}

void SetState::during()
{
    LOG("Calibrating strain gauge offset...");
    _Robot->initPositionControl();
    PAUSE;
    _Robot->setPosition(Eigen::Vector4d::Zero());
    _Robot->calibrateForceSensors();

    LOG("Calibrating strain gauge scale...");
    _Robot->initTorqueControl();
    PAUSE;

    if (_Node->get_dev_toggle().save_default) {
        LOG("Overwriting default parameters...");
        // TODO:
        // std::string asdf;
        // _Node->config_path(asdf);
        // YAML::Node temp = YAML::LoadFile(asdf);
        // std::ofstream os(asdf);
        // os << temp;
    }

    _Node->set_is_saved(true);
}

void SetState::exit()
{
    LOG("<<< Exited <<<");
}