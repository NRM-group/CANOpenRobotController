#include "ExoState.hpp"
// #include "pd.hpp"
#define LOG(x) spdlog::info("[RunState]: {}", x)

typedef ctrl::AdaptiveController<double, X2_NUM_JOINTS, 25> AFFC;

RunState::RunState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Run State"), _Robot(robot), _Node(node),
      _DuringGait{false}, _Position{}, _MinROM{}, _MaxROM{},
      _CtrlExternal{}, _CtrlFriction{}, _CtrlGravity{},
      _CtrlPD{}, _CtrlTorque{}, _CtrlTransparentWalk{}, _LookupTable{100}
{
    _Node->ros_declare(
        {"friction.static",
         "friction.viscous",
         "friction.neg",
         "friction.pos",
         "pd.left_kp",
         "pd.right_kp",
         "pd.left_kd",
         "pd.right_kd",
         "pd.alpha_min",
         "pd.alpha_max",
         "external",
         "torque",
         "l", "m", "s"});
}

void RunState::entry()
{
    LOG(">>> Entered >>>");
    std::vector<double> buffer, buffer2, buffer3, buffer4;

    // External control parameters
    _Node->ros_parameter("external", buffer);
    _CtrlExternal.set_external_torque(buffer);

    // Friction control parameters
    _Node->ros_parameter("friction.static", buffer);
    _CtrlFriction.set_static(buffer);
    _Node->ros_parameter("friction.viscous", buffer);
    _CtrlFriction.set_static(buffer);
    _Node->ros_parameter("friction.neg", buffer);
    _Node->ros_parameter("friction.pos", buffer2);
    _CtrlFriction.set_deadband(buffer, buffer2);

    // PD control parameters
    _Node->ros_parameter("pd.left_kp", buffer);
    _Node->ros_parameter("pd.right_kp", buffer2);
    _Node->ros_parameter("pd.left_kd", buffer3);
    _Node->ros_parameter("pd.right_kd", buffer4);
    _CtrlPD.set_gains(buffer, buffer2, buffer3, buffer4);
    _Node->ros_parameter("pd.alpha_min", buffer);
    _Node->ros_parameter("pd.alpha_max", buffer2);
    _CtrlPD.set_alphas(buffer, buffer2);

    // Torque control parameters
    _Node->ros_parameter("torque", buffer);
    try
    {
        _CtrlTorque.set_gain(buffer);
    }
    catch (const char *msg)
    {
        LOG(msg);
    }

    // Gravity control parameters
    std::vector<double> l, m, s;
    _Node->ros_parameter("l", l);
    _Node->ros_parameter("m", m);
    _Node->ros_parameter("s", s);
    double mass_thigh = m[0] + m[1];
    double mass_shank = m[2] + m[3] + m[4];
    double com_thigh = (s[0] * m[0] + (l[0] - s[1]) * m[1]) / mass_thigh;
    double com_shank = (s[2] * m[2] + (l[1] - s[3]) * m[3] + (l[1] + s[4]) * m[4]) / mass_shank;
    Eigen::Vector4d mass{mass_thigh, mass_shank, mass_thigh, mass_shank};
    Eigen::Vector4d com{com_thigh, com_shank, com_thigh, com_shank};
    _CtrlGravity.set_parameters(mass, {l[0], l[1], l[2], l[3]}, com);

    // LookupTable setup
    std::string csv_file;
    _Node->get_gait_file(csv_file);
    _LookupTable.readCSV(csv_file);
    _LookupTable.resetTrajectory();
    _LookupTable.startTrajectory(_Position, INITIAL_GAIT_RATE);

    // Initialise torque control
    _Robot->initTorqueControl();
    _Node->set_is_saved(false);
}

void RunState::during()
{
    update_controllers();
    update_lookup_table();

    _TorqueOutput = Eigen::Vector4d::Zero();

    // Sum toggled controllers
    // External
    if (_Node->get_dev_toggle().external)
    {
        _TorqueOutput += _CtrlExternal.output();
#ifdef DEBUG
        spdlog::info("external: [{:.4}, {:.4}, {:.4}, {:.4}]",
                     _CtrlExternal.output()[0],
                     _CtrlExternal.output()[1],
                     _CtrlExternal.output()[2],
                     _CtrlExternal.output()[3]);
#endif
    }
    // Friction
    if (_Node->get_dev_toggle().friction)
    {
        _CtrlFriction.loop(_Robot->getVelocity());
        _TorqueOutput += _CtrlFriction.output();
#ifdef DEBUG
        spdlog::info("Friction: [{:.4}, {:.4}, {:.4}, {:.4}]",
                     _CtrlFriction.output()[0],
                     _CtrlFriction.output()[1],
                     _CtrlFriction.output()[2],
                     _CtrlFriction.output()[3]);
#endif
    }
    // Gravity
    if (_Node->get_dev_toggle().gravity)
    {
        _CtrlGravity.loop(_Robot->getPosition());
        _TorqueOutput += _CtrlGravity.output();
#ifdef DEBUG
        spdlog::info("Gravity: [{:.4}, {:.4}, {:.4}, {:.4}]",
                     _CtrlGravity.output()[0],
                     _CtrlGravity.output()[1],
                     _CtrlGravity.output()[2],
                     _CtrlGravity.output()[3]);
#endif
    }
    // PD
    if (_Node->get_dev_toggle().pd && (_Node->get_user_command().toggle_sit | _Node->get_user_command().toggle_walk))
    {
        rate_limit(_LookupTable.getPosition(), _Position, POSITION_RATE);
        _CtrlPD.loop(_Position, _Robot->getPosition());
        _TorqueOutput += _CtrlPD.output();
#ifdef DEBUG
        spdlog::info("PD: [{:.4}, {:.4}, {:.4}, {:.4}]",
                     _CtrlPD.output()[0],
                     _CtrlPD.output()[1],
                     _CtrlPD.output()[2],
                     _CtrlPD.output()[3]);
#endif
    }
    // Torque (transparent)
    if (_Node->get_dev_toggle().torque && !_Node->get_user_command().toggle_walk)
    {
        _CtrlTorque.loop(_Robot->getStrainGauges());
        _TorqueOutput += _CtrlTorque.output();
#ifdef DEBUG
        spdlog::info("Torque: [{:.4}, {:.4}, {:.4}, {:.4}]",
                     _CtrlTorque.output()[0],
                     _CtrlTorque.output()[1],
                     _CtrlTorque.output()[2],
                     _CtrlTorque.output()[3]);
#endif
    }
    // Transparent Walk (transparent)
    if (_Node->get_dev_toggle().torque && _Node->get_user_command().toggle_walk)
    {
        _CtrlTransparentWalk.loop(_Robot->getStrainGauges());
        _TorqueOutput += _CtrlTransparentWalk.output();
#ifdef DEBUG
        spdlog::info("Transparent Walk: [{:.4}, {:.4}, {:.4}, {:.4}]",
                     _CtrlTransparentWalk.output()[0],
                     _CtrlTransparentWalk.output()[1],
                     _CtrlTransparentWalk.output()[2],
                     _CtrlTransparentWalk.output()[3]);
#endif
    }

    // Limit torque output
    for (std::size_t i = 0; i < X2_NUM_JOINTS; i++)
    {
        if (_TorqueOutput[i] > _Node->get_torque_limit())
        {
            _TorqueOutput[i] = _Node->get_torque_limit();
        }
        else if (_TorqueOutput[i] < -_Node->get_torque_limit())
        {
            _TorqueOutput[i] = -_Node->get_torque_limit();
        }
    }
    
    // Set the robot output torques to the desired torque
    _Robot->setTorque(_TorqueOutput);

    // Publish values
    _Node->publish_joint_reference(std::vector<double>(_Position.data(), _Position.data() + _Position.size()));
    _Node->publish_joint_state();
    _Node->publish_strain_gauge();
    _Node->publish_gait_index(_LookupTable.getGaitIndex());
    _Node->publish_error(_CtrlPD.get_err_prev());
    _Node->publish_der_error(_CtrlPD.get_der_prev());
}

void RunState::exit()
{
    LOG("<<< Exited <<<");
}

//TODO: Check rate limit actualy works
void RunState::rate_limit(const Eigen::Vector4d &target, Eigen::Vector4d &current, double rate)
{
    for (std::size_t i = 0; i < X2_NUM_JOINTS; i++)
    {

        if (std::abs(target[i] - current[i]) > rate)
        {
            if (target[i] > current[i])
            {
                current[i] += rate;
            }
            else
            {
                current[i] -= rate;
            }
        }
        else
        {
            current[i] = target[i];
        }
    }
}

void RunState::update_controllers()
{
    // External
    _CtrlExternal.set_external_torque(
        Eigen::Vector4d(_Node->get_external_parameter().torque.cbegin() + 1));
    // Friction
    _CtrlFriction.set_static(
        Eigen::Vector4d(_Node->get_friction_parameter().static_coefficient.cbegin() + 1));
    _CtrlFriction.set_viscous(
        Eigen::Vector4d(_Node->get_friction_parameter().viscous_coefficient.cbegin() + 1));
    // PD
    _CtrlPD.set_alphas(
        Eigen::Vector4d(_Node->get_pd_parameter().alpha_min.cbegin() + 1),
        Eigen::Vector4d(_Node->get_pd_parameter().alpha_max.cbegin() + 1));
    Eigen::Matrix4d kp{}, kd{};
    kp.topLeftCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().left_kp.cbegin() + 1) *
                             _Node->get_gait_parameter().left_loa * 0.01;
    kp.bottomRightCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().right_kp.cbegin() + 1) *
                                 _Node->get_gait_parameter().right_loa * 0.01;
    kd.topLeftCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().left_kd.cbegin() + 1) *
                             _Node->get_gait_parameter().left_loa * 0.01;
    kd.bottomRightCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().right_kd.cbegin() + 1) *
                                 _Node->get_gait_parameter().right_loa * 0.01;
    _CtrlPD.set_gains(kp, kd);
    // Torque
    try{
        _CtrlTorque.set_gain(Eigen::Vector4d(_Node->get_torque_parameter().gain.cbegin() + 1));
    } catch (const char* msg) {
        LOG(msg);
    }
    // Transparent Walk
    try{
        _CtrlTransparentWalk.set_gain(Eigen::Vector4d(_Node->get_torque_parameter().gain.cbegin() + 1));
        // _CtrlTransparentWalk.set_interaction_torque_mask(60.0);
        _CtrlTransparentWalk.set_interaction_torque_mask(_LookupTable.getGaitIndex());
        // _CtrlTransparentWalk.set_direction(_LookupTable.getVelocity(_LookupTable.getGaitIndex()));
        _CtrlTransparentWalk.set_direction_mask(_LookupTable.getGaitIndex());
    } catch (const char* msg) {
        LOG(msg);
    }
}

void RunState::update_lookup_table()
{
    // Toggle start and stop trajectory
    if (_DuringGait)
    {
        if (!_Node->get_user_command().toggle_sit && !_Node->get_user_command().toggle_walk)
        {
            _LookupTable.stop();
            _DuringGait = false;
        }
    }
    else
    {
        if (_Node->get_user_command().toggle_sit)
        {
            _Position = _Robot->getPosition();
            _LookupTable.setTrajectory(_Position, {0, 0, 0, 0}, INITIAL_STAND_RATE);
            _LookupTable.start();
            _DuringGait = true;
        }
        else if (_Node->get_user_command().toggle_walk)
        {
            _Position = _Robot->getPosition();
            _LookupTable.resetTrajectory();
            _LookupTable.startTrajectory(_Position, INITIAL_GAIT_RATE, 45.0); // Start Right leg forward
            //_LookupTable.startTrajectory(_Position, INITIAL_GAIT_RATE);
            _LookupTable.start();
            _DuringGait = true;
        }
    }

    // Set min and max range of motion
    Eigen::Vector4d rom_min, rom_max;
    for (std::size_t i = 0; i < X2_NUM_JOINTS; i++)
    {
        rom_min[i] = deg2rad(_Node->get_gait_parameter().rom_min[i + 1]);
        rom_max[i] = deg2rad(_Node->get_gait_parameter().rom_max[i + 1]);
    }
    rate_limit(rom_min, _MinROM, MIN_ROM_RATE);
    rate_limit(rom_max, _MaxROM, MAX_ROM_RATE);
    _LookupTable.setROM(_MinROM, _MaxROM);

    // Set period of gait trajectory
    _LookupTable.setPeriod(_Node->get_gait_parameter().step_period);

    // Increment gait
    _LookupTable.next();
}