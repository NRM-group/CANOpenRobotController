#include "ExoState.hpp"

RunState::RunState(StateMachine *state_machine,
                   std::shared_ptr<X2Robot> robot)
    : State(state_machine), Node("exo"), _Robot(robot),
    _TorqueOutput{}, _TorqueLimit{}, _StrainGaugeOffset{}, _CtrlButterStrainGauge{},
    _CtrlPD{}, _CtrlExternal{}, _CtrlFriction{}, _CtrlGravity{}, _CtrlTorque{}
{
    // Create publishers / subscribers
    _PubJointState = create_publisher<JointState>("joint_state", 10);
    _PubOutput = create_publisher<Output>("controller_output", 10);
    _SubCorc = create_subscription<Corc>(
        "corc_params", 10, std::bind(&RunState::corc_callback, this, _1)
    );
    _SubExternal = create_subscription<External>(
        "external_params", 10, std::bind(&RunState::external_callback, this,  _1)
    );
    _SubFriction = create_subscription<Friction>(
        "friction_params", 10, std::bind(&RunState::friction_callback, this, _1)
    );
    _SubPD = create_subscription<PD>(
        "pd_params", 10, std::bind(&RunState::pd_callback, this, _1)
    );

    _PubStrainGauge = create_publisher<FloatArray>("strain_gauge", 10);
    _SubStrainGauge = create_subscription<FloatArray>(
        "strain_gauge_scale", 10, std::bind(&RunState::strain_gauge_callback, this, _1)
     );

    _CtrlButterStrainGauge.set_coeff_a(FILTER_COEFF_A);
    _CtrlButterStrainGauge.set_coeff_b(FILTER_COEFF_B);

    // Get parameters
    std::vector<double> m, s;
    std::vector<double> l;
    declare_parameter("m");
    declare_parameter("l");
    declare_parameter("s");
    get_parameter("m", m);
    get_parameter("l", l);
    get_parameter("s", s);

    // Calculate gravity control parameter
    double mass_thigh = m[0] + m[1];
    double mass_shank = m[2] + m[3];
    double com_thigh = (s[0] * m[0] + (l[0] - s[1]) * m[1]) / mass_thigh;
    double com_shank = (s[2] * m[2] + (l[1] - s[3]) * m[3]) / mass_shank;
    Eigen::Vector4d mass { mass_thigh, mass_shank, mass_thigh, mass_shank };
    Eigen::Vector4d com { com_thigh, com_shank, com_thigh, com_shank };
    _CtrlGravity.set_parameters(mass, { l[0], l[1], l[2], l[3] }, com);


    //Retrieve walk cycle csv
    declare_parameter("walking_gait");
    get_parameter("waklking_gait", walkCSV);
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
    spdlog::info("Calibrating strain gauges...");
    sleep(1);
    std::copy(
        _Robot->getJointTorquesViaStrainGauges().data(),
        _Robot->getJointTorquesViaStrainGauges().data() + _Robot->getJointTorquesViaStrainGauges().size(),
        _StrainGaugeOffset.begin()
    );
    spdlog::info("Strain gauge offset [{}, {}, {}, {}]",
        _StrainGaugeOffset[0],
        _StrainGaugeOffset[1],
        _StrainGaugeOffset[2],
        _StrainGaugeOffset[3]
    );
    posReader.readCSV(walkCSV);
    posReader.startTrajectory(_Robot->getPosition(), 0.1);
}

void RunState::during()
{
    static bool once = true;
    if (once)
    {

    once = false;
    }

    //_Robot->initTorqueControl();

    sg[0] = _Robot->getJointTorquesViaStrainGauges()[0] - _StrainGaugeOffset[0];
    sg[1] = _Robot->getJointTorquesViaStrainGauges()[1] - _StrainGaugeOffset[1]; 
    sg[2] = _Robot->getJointTorquesViaStrainGauges()[2] - _StrainGaugeOffset[2]; 
    sg[3] = _Robot->getJointTorquesViaStrainGauges()[3] - _StrainGaugeOffset[3]; 

    sg[0] *= -0.019193478;
    sg[1] *= -0.016627119;
    sg[2] *= 0.015766071;
    sg[3] *= 0.01308;

    //for (unsigned i = 0; i < 4; i++)
    //{
    //    if (sg[i] > 0)
    //    {
    //        sg[i] *= scale[i * 2];
    //    }
    //    else
    //    {
    //        sg[i] *= scale[i * 2 + 1];
    //    }
    //}
    _DesiredPositions = posReader.getNextPos();

    _CtrlButterStrainGauge.filter(Eigen::Vector4d(sg));
    _CtrlPD.loop(_DesiredPositions - _Robot->getPosition());
    _CtrlFriction.loop(_Robot->getVelocity());
    _CtrlGravity.loop(_Robot->getPosition());
    _CtrlTorque.loop(_CtrlButterStrainGauge.output());

    this->calculate_torque();
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

void RunState::calculate_torque()
{
    //_CtrlButter.filter(_CtrlGravity.output());
    //_TorqueOutput =  _CtrlGravity.output();
    //_TorqueOutput << 20,0,20,0;
    _TorqueOutput = _CtrlFriction.output() + _CtrlGravity.output() + _CtrlTorque.output() + _CtrlPD.output();
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
    //std::vector<double> strain_gauge(
    //    _Robot->getJointTorquesViaStrainGauges().data(),
    //    _Robot->getJointTorquesViaStrainGauges().data() + _Robot->getJointTorquesViaStrainGauges().size()
    //);
    //msg.data = strain_gauge;

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
    std::copy(msg->data.begin(), msg->data.begin() + 8, scale);
}