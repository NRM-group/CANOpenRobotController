#include "ExoState.hpp"

RunState::RunState(StateMachine *state_machine,
                   std::shared_ptr<X2Robot> robot)
    : State(state_machine), Node("exo"), _Robot(robot),
    _TorqueLimit{}, _TorqueOutput{}, _StrainGauge{}, _StrainGaugeOffset{},
    _LookupTable{}, _DevToggle{}, _GaitParameter{}, _PatientParameter{},
    _SitToStandParameter{}, _UserCommand{}, _CtrlButterStrainGauge{},
    _CtrlExternal{}, _CtrlFriction{}, _CtrlGravity{}, _CtrlPD{}, _CtrlTorque{}
{
    // Create publishers
    _PubJointState = create_publisher<JointState>("joint_states", 4);
    _PubJointReference = create_publisher<DoubleArray>("joint_references", 4);
    _PubStrainGauge = create_publisher<DoubleArray>("strain_gauges", 4);

    // Create subscribers
    _SubDevToggle = create_subscription<DevToggle>(
        "dev_toggles", 4,
        std::bind(&RunState::dev_toggle_callback, this, _1)
    );
    _SubExternalParameter = create_subscription<ExternalParameter>(
        "external_parameters", 4,
        std::bind(&RunState::external_parameter_callback, this, _1)
    );
    _SubFrictionParameter = create_subscription<FrictionParameter>(
        "friction_parameters", 4,
        std::bind(&RunState::friction_parameter_callback, this, _1)
    );
    _SubGaitParameter = create_subscription<GaitParameter>(
        "gait_parameters", 4,
        std::bind(&RunState::gait_parameter_callback, this, _1)
    );
    _SubMaximumTorque = create_subscription<Double>(
        "maximum_torque", 4,
        std::bind(&RunState::maximum_torque_callback, this, _1)
    );
    _SubPatientParameter = create_subscription<PatientParameter>(
        "patient_parameters", 4,
        std::bind(&RunState::patient_parameter_callback, this, _1)
    );
    _SubPDParameter = create_subscription<PDParameter>(
        "pd_parameters", 4,
        std::bind(&RunState::pd_parameter_callback, this, _1)
    );
    _SubSitToStandParameter = create_subscription<SitToStandParameter>(
        "sit_to_stand_parameters", 4,
        std::bind(&RunState::sit_to_stand_parameter_callback, this, _1)
    );
    _SubUserCommand = create_subscription<UserCommand>(
        "user_commands", 4,
        std::bind(&RunState::user_command_callback, this, _1)
    );

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
    std::copy(
        strain_gauge_scale.begin(),
        strain_gauge_scale.end(),
        _StrainGaugeScale.data()
    );

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

    //Retrieve walk cycle csv
    declare_parameter("walking_gait");
    spdlog::info("RunState: Ready");
}

RunState::~RunState()
{
    spdlog::info("RunState: Destructed");
}

void RunState::entry()
{
    _Robot->initTorqueControl();
    // spdlog::info("RunState: Calibrating strain gauges...");
    // sleep(1);
    // _StrainGaugeOffset = _Robot->get_strain_gauge();
    // spdlog::info("RunState: Strain gauge offset [{}, {}, {}, {}]",
    //     _StrainGaugeOffset[0], _StrainGaugeOffset[1],
    //     _StrainGaugeOffset[2], _StrainGaugeOffset[3]
    // );
    _LookupTable.startTrajectory(_Robot->getPosition(), 4);
    spdlog::info("RunState: Call to entry()");
    spdlog::info("Calibrating strain gauges...");
    sleep(1);
    spdlog::info("Strain gauge offset [{}, {}, {}, {}]",
        _StrainGaugeOffset[0],
        _StrainGaugeOffset[1],
        _StrainGaugeOffset[2],
        _StrainGaugeOffset[3]
    );
}

void RunState::during()
{
    // _TorqueOutput = Eigen::Vector4d::Zero();
    // _StrainGauge = _StrainGaugeScale.cwiseProduct(
        // _Robot->get_strain_gauge() - _StrainGaugeOffset
    // );
//
    // if (_UserCommand.toggle_walk) {
        // _LookupTable.next();
//
        // if (_DevToggle.pd) {
            // _CtrlPD.loop(_LookupTable.getJointPositions() - _Robot->getPosition());
            // _TorqueOutput += _CtrlPD.output();
        // }
    // }
//
    // if (_DevToggle.external) {
        // _TorqueOutput += _CtrlExternal.output();
        // static bool start = true;
        // if (start)
        // {
            // static unsigned long a = 0;
            // if (++a % 200 == 0)
            // {
                // if (_TorqueOutput[0] == -30)
                    // start = false;
                // else
                    // _TorqueOutput += Eigen::Vector4d::Ones();
            // }
        // }
        // else
        // {
            // static unsigned long counter = 0;
            // static Eigen::Vector4d sum{};
            // sum += _StrainGauge;
            // if (++counter % 1000 == 0)
            // {
                // if (_TorqueOutput[0] == 30)
                // {
                    // spdlog::info("FINISHED");
                    // _TorqueOutput = Eigen::Vector4d::Zero();
                // }
                // else
                // {
                    // _TorqueOutput -= Eigen::Vector4d::Ones();
                // }
                // sum /= 1000;
                // spdlog::info("torque [{}] : value [{},{},{},{}]",
                    // _TorqueOutput[0], sum[0], sum[1], sum[2], sum[3]
                // );
                // sum = Eigen::Vector4d::Zero();
            // }
        // }
    // }
    // if (_DevToggle.friction) {
        // _CtrlFriction.loop(_Robot->getVelocity());
        // _TorqueOutput += _CtrlFriction.output();
    // }
    // if (_DevToggle.gravity) {
        // _CtrlGravity.loop(_Robot->getPosition());
        // _TorqueOutput += _CtrlGravity.output();
    // }
    // if (_DevToggle.torque) {
        // _CtrlButterStrainGauge.filter(_StrainGauge);
        // _CtrlTorque.loop(_CtrlButterStrainGauge.output());
        // _TorqueOutput += _CtrlTorque.output();
    // }
//
    // this->limit_torque();
    // _Robot->setTorque(_TorqueOutput);

    // publish_joint_state();
    // publish_joint_reference();
    // publish_strain_gauge();
    // rclcpp::spin_some(get_node_base_interface());
}

void RunState::exit()
{
    _Robot->initTorqueControl();
    spdlog::info("RunState: Call to exit()");
}

void RunState::limit_torque()
{
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++)
    {
        if (_TorqueOutput[i] > _TorqueLimit)
        {
            _TorqueOutput[i] = _TorqueLimit;
        }
        else if (_TorqueOutput[i] < -_TorqueLimit)
        {
            _TorqueOutput[i] = -_TorqueLimit;
        }
    }
}

void RunState::publish_joint_state()
{
    JointState msg;

    static const std::vector<std::string> name = {
        "left_hip", "left_knee", "right_hip", "right_knee"
    };

    msg.header.stamp = now();
    msg.name.assign(name.begin(), name.end());
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

void RunState::publish_joint_reference()
{
    DoubleArray msg;
    msg.data.assign(
        _LookupTable.getJointPositions().data(),
        _LookupTable.getJointPositions().data() + _LookupTable.getJointPositions().size()
    );
    _PubJointReference->publish(msg);
}

void RunState::publish_strain_gauge()
{
    DoubleArray msg;
    msg.data.assign(_StrainGauge.data(), _StrainGauge.data() + _StrainGauge.size());
    _PubStrainGauge->publish(msg);
}

void RunState::dev_toggle_callback(const DevToggle::SharedPtr msg)
{
    _DevToggle = *msg;
}

void RunState::external_parameter_callback(const ExternalParameter::SharedPtr msg)
{
    _CtrlExternal.set_external_torque(Eigen::Vector4d(msg->torque.begin() + 1));
}

void RunState::friction_parameter_callback(const FrictionParameter::SharedPtr msg)
{
    _CtrlFriction.set_static(Eigen::Vector4d(msg->static_coefficient.begin() + 1));
    _CtrlFriction.set_viscous(Eigen::Vector4d(msg->viscous_coefficient.begin() + 1));
}

void RunState::gait_parameter_callback(const GaitParameter::SharedPtr msg)
{
    _GaitParameter = *msg;
    _LookupTable.setPeriod(msg->step_period);
}

void RunState::maximum_torque_callback(const Double::SharedPtr msg)
{
    _TorqueLimit = msg->data;
}

void RunState::patient_parameter_callback(const PatientParameter::SharedPtr msg)
{
    _PatientParameter = *msg;
}

void RunState::pd_parameter_callback(const PDParameter::SharedPtr msg)
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

void RunState::sit_to_stand_parameter_callback(const SitToStandParameter::SharedPtr msg)
{
    _SitToStandParameter = *msg;
}

void RunState::user_command_callback(const UserCommand::SharedPtr msg)
{
    _UserCommand = *msg;
}
