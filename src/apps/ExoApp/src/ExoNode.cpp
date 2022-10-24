#include "ExoNode.hpp"
#define LOG(x)  spdlog::info("[ExoNode]: {}", x)

/***************
 * CONSTRUCTOR *
 ***************/
ExoNode::ExoNode(std::shared_ptr<X2Robot> robot)
    : Node(robot->getRobotName()), _Robot(robot),
    _DevToggle(), _ExternalParameter(), _FrictionParameter(),
    _GaitParameter(), _HeartBeat(), _PatientParameter(),
    _PDParameter(), _SitToStandParameter(), _TorqueLimit(),
    _TorqueParameter(), _UserCommand(), _SaveError(), _IsSaved()
{
    _PubHeartBeat = create_publisher<HeartBeat>("corc_heartbeat", 4);
    _PubJointState = create_publisher<JointState>("joint_states", 4);
    _PubJointReference = create_publisher<FloatArray>("joint_references", 4);
    _PubStrainGauge = create_publisher<FloatArray>("strain_gauges", 4);

    _SubDevToggle = create_subscription<DevToggle>(
        "dev_toggles", 4,
        std::bind(&ExoNode::dev_toggle_callback, this, _1)
    );
    _SubExternalParameter = create_subscription<ExternalParameter>(
        "external_parameters", 4,
        std::bind(&ExoNode::external_parameter_callback, this, _1)
    );
    _SubFrictionParameter = create_subscription<FrictionParameter>(
        "friction_parameters", 4,
        std::bind(&ExoNode::friction_parameter_callback, this, _1)
    );
    _SubGaitParameter = create_subscription<GaitParameter>(
        "gait_parameters", 4,
        std::bind(&ExoNode::gait_parameter_callback, this, _1)
    );
    _SubHeartBeat = create_subscription<HeartBeat>(
        "labview_heartbeat", 4,
        std::bind(&ExoNode::heart_beat_callback, this, _1)
    );
    _SubPatientParameter = create_subscription<PatientParameter>(
        "patient_parameters", 4,
        std::bind(&ExoNode::patient_parameter_callback, this, _1)
    );
    _SubPDParameter = create_subscription<PDParameter>(
        "pd_parameters", 4,
        std::bind(&ExoNode::pd_parameter_callback, this, _1)
    );
    _SubSitToStandParameter = create_subscription<SitToStandParameter>(
        "sit_to_stand_parameters", 4,
        std::bind(&ExoNode::sit_to_stand_parameter_callback, this, _1)
    );
    _SubTorqueLimit = create_subscription<Float>(
        "torque_limit", 4,
        std::bind(&ExoNode::torque_limit_callback, this, _1)
    );
    _SubTorqueParameter = create_subscription<TorqueParameter>(
        "torque_parameters", 4,
        std::bind(&ExoNode::torque_parameter_callback, this, _1)
    );
    _SubUserCommand = create_subscription<UserCommand>(
        "user_commands", 4,
        std::bind(&ExoNode::user_command_callback, this, _1)
    );

    declare_parameter<int>("dry_run", 0);
    declare_parameter<std::string>("exo_file", "");
    declare_parameter<std::string>("gait_file", "");

    declare_parameter<double>("max_torque", 0.0);
    declare_parameter<double>("max_velocity", 0.0);
    declare_parameter<double>("position_limits.max_hip", 0.0);
    declare_parameter<double>("position_limits.min_hip", 0.0);
    declare_parameter<double>("position_limits.max_knee", 0.0);
    declare_parameter<double>("position_limits.min_knee", 0.0);

    get_parameter<double>(
        "max_torque",
        _Robot->getRobotParameters().maxTorque
    );
    get_parameter<double>(
        "max_velocity",
        _Robot->getRobotParameters().maxVelocity
    );
    get_parameter<double>(
        "position_limits.max_hip",
        _Robot->getRobotParameters().jointPositionLimits.hipMax
    );
    get_parameter<double>(
        "position_limits.min_hip",
        _Robot->getRobotParameters().jointPositionLimits.hipMin
    );
    get_parameter<double>(
        "position_limits.max_knee",
        _Robot->getRobotParameters().jointPositionLimits.kneeMax
    );
    get_parameter<double>(
        "position_limits.min_knee",
        _Robot->getRobotParameters().jointPositionLimits.kneeMin
    );

    time_t tnow = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now()
    );
    tm *date = std::localtime(&tnow);
    date->tm_hour = 0;
    date->tm_min = 0;
    date->tm_sec = 0;
    _Midnight = std::chrono::system_clock::from_time_t(std::mktime(date));
}

/***************************
 * APPLICATION EVENT FLAGS *
 ***************************/
bool ExoNode::overwrite_save()
{
    return _DevToggle.save_default;
}

bool ExoNode::save_error()
{
    return _SaveError;
}

bool ExoNode::is_saved()
{
    return _IsSaved;
}

bool ExoNode::ok()
{
    int dry_run;

    get_parameter<int>("dry_run", dry_run);

    return dry_run || (_HeartBeat.status == ExoNode::OK);
}

/**************
 * VISIBILITY *
 **************/
void ExoNode::ros_declare(const std::vector<std::string> &names)
{
    for (const std::string &n : names) {
        declare_parameter<std::vector<double>>(n, { });
    }
}

void ExoNode::ros_parameter(const std::string &name, std::vector<double> &val)
{
    get_parameter<std::vector<double>>(name, val);
}

void ExoNode::get_exo_file(std::string &path)
{
    get_parameter<std::string>("exo_file", path);
}

void ExoNode::get_gait_file(std::string &path)
{
    get_parameter<std::string>("gait_file", path);
}

void ExoNode::set_save_error(bool val)
{
    _SaveError = val;
}

void ExoNode::set_is_saved(bool val)
{
    _IsSaved = val;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ExoNode::get_interface()
{
    return get_node_base_interface();
}

/**************
 * PUBLISHERS *
 **************/
void ExoNode::publish_heart_beat()
{
    HeartBeat msg{};

    auto clock = std::chrono::system_clock::now();
    auto time_since_midnight = clock - _Midnight;
    auto sec_since_midnight = std::chrono::duration_cast<std::chrono::seconds>(clock - _Midnight);

    msg.seconds = sec_since_midnight.count();
    msg.milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        time_since_midnight - sec_since_midnight
    ).count();

    // TODO:
    msg.status = 1;

    _PubHeartBeat->publish(msg);
}

void ExoNode::publish_joint_reference(const std::vector<double> &val)
{
    // TODO: May need to be offset + 1
    FloatArray msg{};

    msg.data = std::move(val);

    _PubJointReference->publish(msg);
}

void ExoNode::publish_joint_state()
{
    JointState msg{};

    const std::array<std::string, X2_NUM_JOINTS> name {
        "left_hip", "left_knee", "right_hip", "right_knee"
    };

    msg.header.stamp = now();
    msg.name.assign(name.cbegin(), name.cend());
    msg.position.assign(
        _Robot->getPosition().data(),
        _Robot->getPosition().data() + _Robot->getPosition().size()
    );
    msg.velocity.assign(
        _Robot->getVelocity().data(),
        _Robot->getVelocity().data() + _Robot->getVelocity().size()
    );
    msg.effort.assign(
        _Robot->getTorque().data(),
        _Robot->getTorque().data() + _Robot->getTorque().size()
    );

    _PubJointState->publish(msg);
}

void ExoNode::publish_strain_gauge()
{
    FloatArray msg{};
    
    msg.data.assign(
        _Robot->getStrainGaugeFilter().begin(),
        _Robot->getStrainGaugeFilter().end()
    );

    _PubStrainGauge->publish(msg);
}

/***********
 * Getters *
 ***********/
const DevToggle & ExoNode::get_dev_toggle() const
{
    return _DevToggle;
}

const ExternalParameter & ExoNode::get_external_parameter() const
{
    return _ExternalParameter;
}

const FrictionParameter & ExoNode::get_friction_parameter() const
{
    return _FrictionParameter;
}

const GaitParameter & ExoNode::get_gait_parameter() const
{
    return _GaitParameter;
}

const HeartBeat & ExoNode::get_heart_beat() const
{
    return _HeartBeat;
}

const PatientParameter & ExoNode::get_patient_parameter() const
{
    return _PatientParameter;
}

const PDParameter & ExoNode::get_pd_parameter() const
{
    return _PDParameter;
}

const SitToStandParameter & ExoNode::get_sit_to_stand_parameter() const
{
    return _SitToStandParameter;
}

const double ExoNode::get_torque_limit() const
{
    return _TorqueLimit;
}

const TorqueParameter & ExoNode::get_torque_parameter() const
{
    return _TorqueParameter;
}

const UserCommand & ExoNode::get_user_command() const
{
    return _UserCommand;
}

/*************
 * CALLBACKS *
 *************/
void ExoNode::dev_toggle_callback(const DevToggle::SharedPtr msg)
{
    _DevToggle = std::move(*msg);
}

void ExoNode::external_parameter_callback(const ExternalParameter::SharedPtr msg)
{
    _ExternalParameter = std::move(*msg);
}

void ExoNode::friction_parameter_callback(const FrictionParameter::SharedPtr msg)
{
    _FrictionParameter = std::move(*msg);
}

void ExoNode::gait_parameter_callback(const GaitParameter::SharedPtr msg)
{
    _GaitParameter = std::move(*msg);
}

void ExoNode::heart_beat_callback(const HeartBeat::SharedPtr msg)
{
    _HeartBeat = std::move(*msg);
}

void ExoNode::patient_parameter_callback(const PatientParameter::SharedPtr msg)
{
    _PatientParameter = std::move(*msg);
}

void ExoNode::pd_parameter_callback(const PDParameter::SharedPtr msg)
{
    _PDParameter = std::move(*msg);
}

void ExoNode::sit_to_stand_parameter_callback(const SitToStandParameter::SharedPtr msg)
{
    _SitToStandParameter = std::move(*msg);
}

void ExoNode::torque_limit_callback(const Float::SharedPtr msg)
{
    _TorqueLimit = msg->data;
}

void ExoNode::torque_parameter_callback(const TorqueParameter::SharedPtr msg)
{
    _TorqueParameter = std::move(*msg);
}

void ExoNode::user_command_callback(const UserCommand::SharedPtr msg)
{
    _UserCommand = std::move(*msg);
}