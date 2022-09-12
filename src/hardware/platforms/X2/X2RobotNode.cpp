#include "X2Robot.h"

X2Robot::X2Robot() : Robot(X2_NUM_JOINTS), Node("exo"),
    _TorqueLimit{}, _TorqueOutput{}, _StrainGauge{}, _StrainGaugeOffset{},
    _LookupTable{}, _DevToggle{}, _GaitParameter{}, _PatientParameter{},
    _SitToStandParameter{}, _UserCommand{}, _CtrlButterStrainGauge{},
    _CtrlExternal{}, _CtrlFriction{}, _CtrlGravity{}, _CtrlPD{}, _CtrlTorque{}
{
    spdlog::info("X2Robot: Initialising robot...(1/4)");
    init();

    spdlog::info("X2Robot: Creating publishers...(2/4)");
    _PubJointState = create_publisher<JointState>("joint_states", 4);
    _PubJointReference = create_publisher<DoubleArray>("joint_references", 4);
    _PubStrainGauge = create_publisher<DoubleArray>("strain_gauges", 4);

    spdlog::info("X2Robot: Creating subscriptions...(3/4)");
    _SubDevToggle = create_subscription<DevToggle>(
        "dev_toggles", 4,
        std::bind(&X2Robot::dev_toggle_callback, this, _1)
    );
    _SubExternalParameter = create_subscription<ExternalParameter>(
        "external_parameters", 4,
        std::bind(&X2Robot::external_parameter_callback, this, _1)
    );
    _SubFrictionParameter = create_subscription<FrictionParameter>(
        "friction_parameters", 4,
        std::bind(&X2Robot::friction_parameter_callback, this, _1)
    );
    _SubGaitParameter = create_subscription<GaitParameter>(
        "gait_parameters", 4,
        std::bind(&X2Robot::gait_parameter_callback, this, _1)
    );
    _SubMaximumTorque = create_subscription<Double>(
        "maximum_torque", 4,
        std::bind(&X2Robot::maximum_torque_callback, this, _1)
    );
    _SubPatientParameter = create_subscription<PatientParameter>(
        "patient_parameters", 4,
        std::bind(&X2Robot::patient_parameter_callback, this, _1)
    );
    _SubPDParameter = create_subscription<PDParameter>(
        "pd_parameters", 4,
        std::bind(&X2Robot::pd_parameter_callback, this, _1)
    );
    _SubSitToStandParameter = create_subscription<SitToStandParameter>(
        "sit_to_stand_parameters", 4,
        std::bind(&X2Robot::sit_to_stand_parameter_callback, this, _1)
    );
    _SubUserCommand = create_subscription<UserCommand>(
        "user_commands", 4,
        std::bind(&X2Robot::user_command_callback, this, _1)
    );

    spdlog::info("X2Robot: Retrieving parameters...(4/4)");
    // Hardware parameters
    declare_parameter("hip_max");
    declare_parameter("hip_min");
    declare_parameter("knee_max");
    declare_parameter("knee_max");
    declare_parameter("max_torque");
    declare_parameter("max_velocity");
    get_parameter("hip_max", _MaxHip);
    get_parameter("hip_min", _MinHip);
    get_parameter("knee_max", _MaxKnee);
    get_parameter("knee_min", _MinKnee);
    get_parameter("torque_max", _MaxTorque);
    get_parameter("velocity_max", _MaxVelocity);
    // Lookup table parameter
    std::string gait_file;
    declare_parameter("gait_file");
    get_parameter("gait_file", gait_file);
    _LookupTable.readCSV(gait_file);
    // Strain gauge filter parameters
    std::vector<double> filter_coeff_a, filter_coeff_b;
    declare_parameter("filter_coeff_a");
    declare_parameter("filter_coeff_b");
    get_parameter("filter_coeff_a", filter_coeff_a);
    get_parameter("filter_coeff_b", filter_coeff_b);
    std::array<double, FILTER_ORDER + 1> coeff_a, coeff_b;
    std::copy(filter_coeff_a.begin(), filter_coeff_a.end(), coeff_a.begin());
    std::copy(filter_coeff_b.begin(), filter_coeff_b.end(), coeff_b.begin());
    _CtrlButterStrainGauge.set_coeff_a(coeff_a);
    _CtrlButterStrainGauge.set_coeff_b(coeff_b);
    // Strain gauge scales
    std::vector<double> strain_gauge_scale;
    declare_parameter("strain_gauge_scale");
    get_parameter("strain_gauge_scale", strain_gauge_scale);
    std::copy(strain_gauge_scale.begin(), strain_gauge_scale.end(), _StrainGaugeScale.data());
    // Controller parameters
    std::vector<double> m, s;
    std::vector<double> l;
    declare_parameter("m");
    declare_parameter("l");
    declare_parameter("s");
    get_parameter("m", m);
    get_parameter("l", l);
    get_parameter("s", s);
    // Gravity control parameter
    double mass_thigh = m[0] + m[1];
    double mass_shank = m[2] + m[3] + m[4];
    double com_thigh = (s[0] * m[0] + (l[0] - s[1]) * m[1]) / mass_thigh;
    double com_shank = (s[2] * m[2] + (l[1] - s[3]) * m[3] + (l[1] + s[4]) * m[4]) / mass_shank;
    Eigen::Vector4d mass { mass_thigh, mass_shank, mass_thigh, mass_shank };
    Eigen::Vector4d com { com_thigh, com_shank, com_thigh, com_shank };
    _CtrlGravity.set_parameters(mass, { l[0], l[1], l[2], l[3] }, com);

    spdlog::info("X2Robot: Ready");
}

X2Robot::~X2Robot()
{
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        delete inputs[i];
        delete joints[i];
    }
    spdlog::info("X2Robot: Destruct");
}

void X2Robot::update_node()
{
    publish_joint_state();
    publish_joint_reference();
    publish_strain_gauge();
    rclcpp::spin_some(this->get_node_base_interface());
}

void X2Robot::publish_joint_state()
{
    JointState msg;

    static const std::vector<std::string> name = {
        "left_hip", "left_knee", "right_hip", "right_knee"
    };

    msg.header.stamp = now();
    msg.name.assign(name.begin(), name.end());
    msg.position.assign(
        jointPositions_.data(),
        jointPositions_.data() + jointPositions_.size()
    );
    msg.velocity.assign(
        jointVelocities_.data(),
        jointVelocities_.data() + jointVelocities_.size()
    );
    msg.effort.assign(
        jointTorques_.data(),
        jointTorques_.data() + jointTorques_.size()
    );

    _PubJointState->publish(msg);
}

void X2Robot::publish_joint_reference()
{
    DoubleArray msg;
    msg.data.assign(
        _LookupTable.getJointPositions().data(),
        _LookupTable.getJointPositions().data() + _LookupTable.getJointPositions().size()
    );
    _PubJointReference->publish(msg);
}

void X2Robot::publish_strain_gauge()
{
    DoubleArray msg;
    msg.data.assign(_StrainGauge.data(), _StrainGauge.data() + _StrainGauge.size());
    _PubStrainGauge->publish(msg);
}

void X2Robot::dev_toggle_callback(const DevToggle::SharedPtr msg)
{
    _DevToggle = *msg;
}

void X2Robot::external_parameter_callback(const ExternalParameter::SharedPtr msg)
{
    _CtrlExternal.set_external_torque(Eigen::Vector4d(msg->torque.begin() + 1));
}

void X2Robot::friction_parameter_callback(const FrictionParameter::SharedPtr msg)
{
    _CtrlFriction.set_static(Eigen::Vector4d(msg->static_coefficient.begin() + 1));
    _CtrlFriction.set_viscous(Eigen::Vector4d(msg->viscous_coefficient.begin() + 1));
}

void X2Robot::gait_parameter_callback(const GaitParameter::SharedPtr msg)
{
    _GaitParameter = *msg;
    _LookupTable.setPeriod(msg->step_period);
}

void X2Robot::maximum_torque_callback(const Double::SharedPtr msg)
{
    _TorqueLimit = msg->data;
}

void X2Robot::patient_parameter_callback(const PatientParameter::SharedPtr msg)
{
    _PatientParameter = *msg;
}

void X2Robot::pd_parameter_callback(const PDParameter::SharedPtr msg)
{
    Eigen::Matrix4d kp, kd;

    kp << msg->left_kp[1], msg->left_kp[2], 0, 0,
          msg->left_kp[3], msg->left_kp[4], 0, 0,
          0, 0, msg->right_kp[1], msg->right_kp[2],
          0, 0, msg->right_kp[3], msg->right_kp[4];
    kd << msg->left_kd[1], msg->left_kd[2], 0, 0,
          msg->left_kd[3], msg->left_kd[4], 0, 0,
          0, 0, msg->right_kd[1], msg->right_kd[2],
          0, 0, msg->right_kd[3], msg->right_kd[4];

    _CtrlPD.set_gains(kp, kd);
    _CtrlPD.set_alphas(
        Eigen::Vector4d(msg->alpha_min.begin() + 1),
        Eigen::Vector4d(msg->alpha_max.begin() + 1)
    );
}

void X2Robot::sit_to_stand_parameter_callback(const SitToStandParameter::SharedPtr msg)
{
    _SitToStandParameter = *msg;
}

void X2Robot::user_command_callback(const UserCommand::SharedPtr msg)
{
    _UserCommand = *msg;
}