#include "ExoState.hpp"

RunState::RunState(StateMachine *state_machine,
                   std::shared_ptr<X2Robot> robot)
    : State(state_machine), Node("exo"), _Robot(robot),
    _TorqueOutput{}, _TorqueLimit{}, _Butterworth{},
    _CtrlPD{}, _CtrlExternal{}, _CtrlFriction{}, _CtrlGravity{}, _CtrlTorque{}
{
    _PubJointState = create_publisher<JointState>("joint_state", 10);
    _PubOutput = create_publisher<Output>("controller_output", 10);
    _SubCorc = create_subscription<Corc>(
        "corc", 10, std::bind(&RunState::corc_callback, this, _1)
    );
    _SubExternal = create_subscription<External>(
        "external", 10, std::bind(&RunState::external_callback, this,  _1)
    );
    _SubFriction = create_subscription<Friction>(
        "friction", 10, std::bind(&RunState::friction_callback, this, _1)
    );
    _SubPD = create_subscription<PD>(
        "pd", 10, std::bind(&RunState::pd_callback, this, _1)
    );


    _PubStrainGauge = create_publisher<FloatArray>("strain_gauge", 10);
    _SubStrainGauge = create_subscription<FloatArray>(
        "strain_gauge_scale", 10, std::bind(&RunState::strain_gauge_callback, this, _1)
    );


    std::vector<double> m, s;
    std::vector<double> l;
    declare_parameter("m");
    declare_parameter("l");
    declare_parameter("s");
    get_parameter("m", m);
    get_parameter("l", l);
    get_parameter("s", s);
    Eigen::Vector4d mass, dist;
    double mass_thigh = m[0] + m[1];
    double mass_shank = m[2] + m[3];
    double com_thigh = (s[0]*m[0] + (l[0]-s[1])*m[1]) / mass_thigh;
    double com_shank = (s[2]*m[2] + (l[1]-s[3])*m[3]) / mass_shank;
    mass << mass_thigh, mass_shank, mass_thigh, mass_shank;
    dist << com_thigh, com_shank, com_thigh, com_shank;
    _CtrlGravity.set_mass(mass);
    _CtrlGravity.set_dist(dist);

    spdlog::info("RunState: Ready");
}

RunState::~RunState()
{
    spdlog::info("RunState: Destructed");
}

void RunState::entry()
{
    _Robot->initTorqueControl();
    spdlog::info("RunState: Call to entry()");
}

void RunState::during()
{
    _Robot->initTorqueControl();
    spdlog::info("----------------------------------");

    sg[0] = _Robot->getJointTorquesViaStrainGauges()[0] - 834;
    sg[1] = _Robot->getJointTorquesViaStrainGauges()[1] - 623;
    sg[2] = _Robot->getJointTorquesViaStrainGauges()[2] - 1012;
    sg[3] = _Robot->getJointTorquesViaStrainGauges()[3] - 571;

    for (unsigned i = 0; i < 4; i++)
    {
        if (sg[i] > 0)
        {
            sg[i] *= scale[i * 2];
        }
        else
        {
            sg[i] *= scale[i * 2 + 1];
        }
    }

    static Eigen::Vector4d offset { M_PI_2, M_PI_2, M_PI_2, M_PI_2};
    _CtrlFriction.loop(_Robot->getVelocity());
    _CtrlGravity.loop(_Robot->getPosition() - offset);
    _CtrlTorque.loop(Eigen::Vector4d(sg));

    _TorqueOutput = _CtrlFriction.output() + _CtrlGravity.output() + _CtrlTorque.output();
    limit_torque();

    _Robot->setTorque(_TorqueOutput);

    publish_joint_state();
    publish_output();
    publish_strain_gauge();
    rclcpp::spin_some(get_node_base_interface());
}

void RunState::exit()
{
    _Robot->initTorqueControl();
    _Robot->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
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

    static std::vector<std::string> name = { // TODO:
        "left_hip", "left_knee", "right_hip", "right_knee"
    };
    std::vector<double> position(
        _Robot->getPosition().data(),
        _Robot->getPosition().data() + _Robot->getPosition().size()
    );
    std::vector<double> velocity(
        _Robot->getVelocity().data(),
        _Robot->getVelocity().data() + _Robot->getVelocity().size()
    );
    std::vector<double> torque(
        _Robot->getTorque().data(),
        _Robot->getTorque().data() + _Robot->getTorque().size()
    );

    msg.header.stamp = now();
    msg.name = name;
    msg.position = position;
    msg.velocity = velocity;
    msg.effort = torque;

    _PubJointState->publish(msg);
}

void RunState::publish_output()
{
    Output msg;

    std::copy(_CtrlPD.begin(), _CtrlPD.end(), msg.pd.begin());
    std::copy(_CtrlExternal.begin(), _CtrlExternal.end(), msg.external.begin());
    std::copy(_CtrlFriction.begin(), _CtrlFriction.end(), msg.friction.begin());
    std::copy(_CtrlGravity.begin(), _CtrlGravity.end(), msg.gravity.begin());
    std::copy(
        _TorqueOutput.data(),
        _TorqueOutput.data() + _TorqueOutput.size(),
        msg.total.begin()
    );

    _PubOutput->publish(msg);
}

void RunState::publish_strain_gauge()
{
    FloatArray msg;

    msg.data.assign(sg, sg + 4);

    _PubStrainGauge->publish(msg);
}

void RunState::corc_callback(const Corc::SharedPtr msg)
{
    _TorqueLimit = msg->maximum_torque;
}

void RunState::external_callback(const External::SharedPtr msg)
{
}

void RunState::friction_callback(const Friction::SharedPtr msg)
{
    _CtrlFriction.set_static(Eigen::Vector4d(msg->b_static.data()));
    _CtrlFriction.set_viscous(Eigen::Vector4d(msg->b_viscous.data()));
}

void RunState::pd_callback(const PD::SharedPtr msg)
{
}

void RunState::strain_gauge_callback(const FloatArray::SharedPtr msg)
{
    std::copy(msg->data.begin(), msg->data.end(), scale);
}