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
    PAUSE;
    _Robot->calibrateForceSensors();

    if (_Node->get_dev_toggle().save_default) {
        LOG("Overwriting default parameters...");
        using vec = std::vector<double>;
        std::string filepath;
        _Node->get_exo_file(filepath);
        YAML::Node config = YAML::LoadFile(filepath);
        auto param = config["exo"]["ros__parameters"];

        // length
        param["l"].as<vec>() = {
            _Node->get_patient_parameter().left_thigh_length,
            _Node->get_patient_parameter().left_shank_length,
            _Node->get_patient_parameter().right_thigh_length,
            _Node->get_patient_parameter().right_thigh_length
        };

        // external
        param["external"].as<vec>() = {
            _Node->get_external_parameter().torque[1],
            _Node->get_external_parameter().torque[2],
            _Node->get_external_parameter().torque[3],
            _Node->get_external_parameter().torque[4],
        };

        // friction
        param["friction"]["static"].as<vec>() = {
            _Node->get_friction_parameter().static_coefficient[1],
            _Node->get_friction_parameter().static_coefficient[2],
            _Node->get_friction_parameter().static_coefficient[3],
            _Node->get_friction_parameter().static_coefficient[4]
        };
        param["friction"]["viscous"].as<vec>() = {
            _Node->get_friction_parameter().viscous_coefficient[1],
            _Node->get_friction_parameter().viscous_coefficient[2],
            _Node->get_friction_parameter().viscous_coefficient[3],
            _Node->get_friction_parameter().viscous_coefficient[4]
        };

        // PD
        param["left_kp"].as<vec>() = {
            _Node->get_pd_parameter().left_kp[1],
            _Node->get_pd_parameter().left_kp[2],
            _Node->get_pd_parameter().left_kp[3],
            _Node->get_pd_parameter().left_kp[4]
        };
        param["right_kp"].as<vec>() = {
            _Node->get_pd_parameter().right_kp[1],
            _Node->get_pd_parameter().right_kp[2],
            _Node->get_pd_parameter().right_kp[3],
            _Node->get_pd_parameter().right_kp[4]
        };
        param["left_kd"].as<vec>() = {
            _Node->get_pd_parameter().left_kd[1],
            _Node->get_pd_parameter().left_kd[2],
            _Node->get_pd_parameter().left_kd[3],
            _Node->get_pd_parameter().left_kd[4]
        };
        param["right_kd"].as<vec>() = {
            _Node->get_pd_parameter().right_kd[1],
            _Node->get_pd_parameter().right_kd[2],
            _Node->get_pd_parameter().right_kd[3],
            _Node->get_pd_parameter().right_kd[4]
        };
        param["alpha_min"].as<vec>() = {
            _Node->get_pd_parameter().alpha_min[1],
            _Node->get_pd_parameter().alpha_min[2],
            _Node->get_pd_parameter().alpha_min[3],
            _Node->get_pd_parameter().alpha_min[4]
        };
        param["alpha_max"].as<vec>() = {
            _Node->get_pd_parameter().alpha_max[1],
            _Node->get_pd_parameter().alpha_max[2],
            _Node->get_pd_parameter().alpha_max[3],
            _Node->get_pd_parameter().alpha_max[4]
        };

        std::ofstream os(filepath), backupos(filepath + ".bak");
        os << config;
        backupos << config;
    }

    _Node->set_is_saved(true);
}

void SetState::exit()
{
    LOG("<<< Exited <<<");
}