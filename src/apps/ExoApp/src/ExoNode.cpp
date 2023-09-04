#include "ExoNode.hpp"
#include "ExoState.hpp"
//#include "pd.hpp"
#define LOG(x)  spdlog::info("[ExoNode]: {}", x)


/***************
 * CONSTRUCTOR *
 ***************/
ExoNode::ExoNode(std::shared_ptr<X2Robot> robot)
    : Node(robot->getRobotName()), _Robot(robot),
    _SaveError(), _IsSaved(), _DevToggle(),
    _ExternalParameter(), _FrictionParameter(),
    _GaitParameter(), _HeartBeat(), _PatientParameter(),
    _PDParameter(), _SitToStandParameter(), _TorqueLimit(),
    _TorqueParameter(), _UserCommand()
{   
    _PubHeartBeat = create_publisher<HeartBeat>("corc_heartbeat", 4);
    _PubJointState = create_publisher<JointState>("joint_states", 4);
    _PubJointReference = create_publisher<FloatArray>("joint_references", 4);
    _PubStrainGauge = create_publisher<FloatArray>("strain_gauges", 4);
    _PubAffcTorque = create_publisher<FloatArray>("affc_torque", 4);

    //*****Code by Alex Anchivilca for Thesis
    _PubErrorLH = create_publisher<Float>("errorLH",2); 
    _PubErrorLK = create_publisher<Float>("errorLK",2); 
    _PubErrorRH = create_publisher<Float>("errorRH",2); 
    _PubErrorRK = create_publisher<Float>("errorRK",2); 

    _PubDerErrorLH = create_publisher<Float>("der_errorLH",2); 
    _PubDerErrorLK = create_publisher<Float>("der_errorLK",2); 
    _PubDerErrorRH = create_publisher<Float>("der_errorRH",2); 
    _PubDerErrorRK = create_publisher<Float>("der_errorRK",2); 

    _PubGaitIndex = create_publisher<Float>("gait_index",2);
    _PubLH_Ref = create_publisher<Float>("lh_ref",2);
    _PubLK_Ref = create_publisher<Float>("lk_ref",2);
    _PubRH_Ref = create_publisher<Float>("rh_ref",2);
    _PubRK_Ref = create_publisher<Float>("rk_ref",2);
    //********
    
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
    declare_parameter<std::string>("exo_path", "");
    declare_parameter<std::string>("gait_path", "");

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

    _HeartBeat.status = 1;
    _GaitParameter.step_period = 10;
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
    get_parameter<std::string>("exo_path", path);
}

void ExoNode::get_gait_file(std::string &path)
{
    get_parameter<std::string>("gait_path", path);
}

void ExoNode::get_affc_file(std::string &path)
{
    get_parameter<std::string>("affc_path", path);
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

    msg.status = HeartBeatStatus::OK;

    if (_Robot->getButtonValue(ButtonColor::RED)) {
        msg.status = HeartBeatStatus::ESTOP;
    }

    // FIXME: HOW TO DETECT CAN BUS DISCONNECT?
    // TODO: Add internal conflict detection?

    _PubHeartBeat->publish(msg);
}

void ExoNode::publish_joint_reference(const std::vector<double> &val)
{
    FloatArray msg{};
    Float lhData{};
    Float lkData{};
    Float rhData{};
    Float rkData{};

    msg.data = std::move(val);
    lhData.data = val[0];
    lkData.data = val[1];
    rhData.data = val[2];
    rkData.data = val[3];    
    _PubJointReference->publish(msg);
    _PubLH_Ref->publish(lhData);
    _PubLK_Ref->publish(lkData);
    _PubRH_Ref->publish(rhData);
    _PubRK_Ref->publish(rkData);
}

void ExoNode::publish_gait_index(double gaitIndex)
{
    Float msg{};
    msg.data = gaitIndex;
    _PubGaitIndex->publish(msg);
}

void ExoNode::publish_affc_torque(const std::vector<double> &val)
{
    FloatArray msg{};

    msg.data = std::move(val);

    _PubAffcTorque->publish(msg);
}

void ExoNode::publish_error(const Eigen::Matrix<double, 4, 1> &error) 
{
    Float lhErrorMsg{};
    Float lkErrorMsg{};
    Float rhErrorMsg{};
    Float rkErrorMsg{};
    lhErrorMsg.data = error[0];
    lkErrorMsg.data = error[1];
    rhErrorMsg.data = error[2];
    rkErrorMsg.data = error[3];    
    _PubErrorLH->publish(lhErrorMsg);
    _PubErrorLK->publish(lkErrorMsg);
    _PubErrorRH->publish(rhErrorMsg);
    _PubErrorRK->publish(rkErrorMsg);
}

void ExoNode::publish_der_error(const Eigen::Matrix<double, 4, 1> &derError) 
{
    Float lhDerErrorMsg{};
    Float lkDerErrorMsg{};
    Float rhDerErrorMsg{};
    Float rkDerErrorMsg{};
    lhDerErrorMsg.data = derError[0];
    lkDerErrorMsg.data = derError[1];
    rhDerErrorMsg.data = derError[2];
    rkDerErrorMsg.data = derError[3];
    _PubDerErrorLH->publish(lhDerErrorMsg);
    _PubDerErrorLK->publish(lkDerErrorMsg);
    _PubDerErrorRH->publish(rhDerErrorMsg);
    _PubDerErrorRK->publish(rkDerErrorMsg);
}

void ExoNode::publish_joint_state()
{
    JointState msg{};

    const std::array<std::string, X2_NUM_JOINTS + 1> name {
        "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint", "world_to_backpack"
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

    msg.position.push_back(_Robot->getBackPackAngleOnMedianPlane());

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
DevToggle & ExoNode::get_dev_toggle()
{
    return _DevToggle;
}

ExternalParameter & ExoNode::get_external_parameter()
{
    return _ExternalParameter;
}

FrictionParameter & ExoNode::get_friction_parameter()
{
    return _FrictionParameter;
}

GaitParameter & ExoNode::get_gait_parameter()
{
    return _GaitParameter;
}

HeartBeat & ExoNode::get_heart_beat()
{
    return _HeartBeat;
}

PatientParameter & ExoNode::get_patient_parameter()
{
    return _PatientParameter;
}

PDParameter & ExoNode::get_pd_parameter()
{
    return _PDParameter;
}

SitToStandParameter & ExoNode::get_sit_to_stand_parameter()
{
    return _SitToStandParameter;
}

double ExoNode::get_torque_limit()
{
    return _TorqueLimit;
}

TorqueParameter & ExoNode::get_torque_parameter()
{
    return _TorqueParameter;
}

UserCommand & ExoNode::get_user_command()
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