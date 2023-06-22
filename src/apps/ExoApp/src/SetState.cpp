#include "ExoState.hpp"
#define LOG(x) spdlog::info("[SetState]: {}", x)
#define ERR(x) spdlog::error("[SetState]: {}", x)

SetState::SetState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Set State"), _Robot(robot), _Node(node)
{
    _Node->ros_declare(
        {
            "strain_gauge.coeff_a",
            "strain_gauge.coeff_b"
        }
    );
}

void SetState::entry()
{
    LOG(">>> Entered >>>");

    std::vector<double> buffer;
    std::array<double, STRAIN_GAUGE_FILTER_ORDER + 1> coeff;

    _Node->ros_parameter("strain_gauge.coeff_a", buffer);
    std::copy(buffer.begin(), buffer.end(), coeff.begin());
    _Robot->getStrainGaugeFilter().set_coeff_a(coeff);

    _Node->ros_parameter("strain_gauge.coeff_b", buffer);
    std::copy(buffer.begin(), buffer.end(), coeff.begin());
    _Robot->getStrainGaugeFilter().set_coeff_b(coeff);

    _Robot->initTorqueControl();
}

void SetState::during()
{
    LOG("Calibrating strain gauge offset...");
    _Robot->setTorque(Eigen::Vector4d::Zero());
    usleep(1e6);
    _Robot->calibrateForceSensors();
    _Robot

    if (_Node->get_dev_toggle().save_default) {

        LOG("Overwriting default parameters...");
        _Node->get_dev_toggle().save_default = false;

        std::string filepath;
        _Node->get_exo_file(filepath);

        YAML::Node param = YAML::LoadFile(filepath)["exo"]["ros__parameters"];

        std::ofstream os(filepath);
        YAML::Emitter config(os);
        config.SetDoublePrecision(6);

        config << YAML::BeginMap;
        config << YAML::Key << "exo";

        config << YAML::BeginMap;
        config << YAML::Key << "ros__parameters";

        config << YAML::BeginMap;

        // Masses
        config << YAML::Key << "m" << YAML::Flow << std::vector<double>
        {
            _Node->get_patient_parameter().left_thigh_length,
            _Node->get_patient_parameter().left_shank_length,
            _Node->get_patient_parameter().right_thigh_length,
            _Node->get_patient_parameter().right_shank_length,
            0.0,
            10.3
        };

        // Lengths
        config << YAML::Key << "l" << YAML::Flow << param["l"].as<std::vector<double>>();

        // Centre of mass
        config << YAML::Key << "s" << YAML::Flow << param["s"].as<std::vector<double>>();

        // External
        config << YAML::Key << "external" << YAML::Flow << std::vector<double>(
            _Node->get_external_parameter().torque.begin() + 1,
            _Node->get_external_parameter().torque.end()
        );

        // Friction
        config << YAML::Key << "friction";
        config << YAML::BeginMap;
        config << YAML::Key << "static" << YAML::Flow << std::vector<double>(
            _Node->get_friction_parameter().static_coefficient.begin() + 1,
            _Node->get_friction_parameter().static_coefficient.end()
        );
        config << YAML::Key << "viscous" << YAML::Flow << std::vector<double>(
            _Node->get_friction_parameter().viscous_coefficient.begin() + 1,
            _Node->get_friction_parameter().viscous_coefficient.end()
        );
        config << YAML::Key << "neg" << YAML::Flow << param["friction"]["neg"].as<std::vector<double>>();
        config << YAML::Key << "pos" << YAML::Flow << param["friction"]["pos"].as<std::vector<double>>();
        config << YAML::EndMap;

        // PD
        config << YAML::Key << "pd";
        config << YAML::BeginMap;
        config << YAML::Key << "left_kp" << YAML::Flow << std::vector<double>(
            _Node->get_pd_parameter().left_kp.begin() + 1,
            _Node->get_pd_parameter().left_kp.end()
        );
        config << YAML::Key << "right_kp" << YAML::Flow << std::vector<double>(
            _Node->get_pd_parameter().left_kp.begin() + 1,
            _Node->get_pd_parameter().left_kp.end()
        );
        config << YAML::Key << "left_kd" << YAML::Flow << std::vector<double>(
            _Node->get_pd_parameter().left_kp.begin() + 1,
            _Node->get_pd_parameter().left_kp.end()
        );
        config << YAML::Key << "right_kd" << YAML::Flow << std::vector<double>(
            _Node->get_pd_parameter().left_kp.begin() + 1,
            _Node->get_pd_parameter().left_kp.end()
        );
        config << YAML::Key << "alpha_min" << YAML::Flow << std::vector<double>(
            _Node->get_pd_parameter().alpha_min.begin() + 1,
            _Node->get_pd_parameter().alpha_min.end()
        );
        config << YAML::Key << "alpha_max" << YAML::Flow << std::vector<double>(
            _Node->get_pd_parameter().alpha_max.begin() + 1,
            _Node->get_pd_parameter().alpha_max.end()
        );
        config << YAML::EndMap;

        // Torque
        // TODO: Implement
        config << YAML::Key << "torque" << YAML::Flow << param["torque"].as<std::vector<double>>();

        // Strain gauge
        config << YAML::Key << "strain_gauge";
        config << YAML::BeginMap;
        config << YAML::Key << "coeff_a" << YAML::Flow << param["strain_gauge"]["coeff_a"].as<std::vector<double>>();
        config << YAML::Key << "coeff_b" << YAML::Flow << param["strain_gauge"]["coeff_b"].as<std::vector<double>>();
        config << YAML::EndMap;

        // Limits
        config << YAML::Key << "max_torque" << YAML::Value << param["max_torque"].as<double>();
        config << YAML::Key << "max_velocity" << YAML::Value << param["max_velocity"].as<double>();
        config << YAML::Key << "position_limits";
        config << YAML::BeginMap;
        config << YAML::Key << "max_hip" << YAML::Value << param["position_limits"]["max_hip"].as<double>();
        config << YAML::Key << "min_hip" << YAML::Value << param["position_limits"]["min_hip"].as<double>();
        config << YAML::Key << "max_knee" << YAML::Value << param["position_limits"]["max_knee"].as<double>();
        config << YAML::Key << "min_knee" << YAML::Value << param["position_limits"]["min_knee"].as<double>();
        config << YAML::EndMap;

        config << YAML::EndMap;
        config << YAML::EndMap;
    }

    _Node->set_is_saved(true);
}

void SetState::exit()
{
    LOG("<<< Exited <<<");
}