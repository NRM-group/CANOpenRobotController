#include "ExoState.hpp"
#define LOG(x)      spdlog::info("[RunState]: {}", x)

RunState::RunState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Run State"), _Robot(robot), _Node(node),
    _CtrlExternal{}, _CtrlFriction{}, _CtrlGravity{},
    _CtrlPD{}, _CtrlTorque{}, _LookupTable{}
{
    _Node->ros_declare(
        {
            "friction.static",
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
            "l", "m", "s"
        }
    );
}

void RunState::entry()
{
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
    _CtrlTorque.set_gain(buffer);

    // Gravity control parameters
    std::vector<double> l, m, s;
    _Node->ros_parameter("l", l);
    _Node->ros_parameter("m", m);
    _Node->ros_parameter("s", s);
    double mass_thigh = m[0] + m[1];
    double mass_shank = m[2] + m[3] + m[4];
    double com_thigh = (s[0] * m[0] + (l[0] - s[1]) * m[1]) / mass_thigh;
    double com_shank = (s[2] * m[2] + (l[1] - s[3]) * m[3] + (l[1] + s[4]) * m[4]) / mass_shank;
    Eigen::Vector4d mass { mass_thigh, mass_shank, mass_thigh, mass_shank };
    Eigen::Vector4d com { com_thigh, com_shank, com_thigh, com_shank };
    _CtrlGravity.set_parameters(mass, { l[0], l[1], l[2], l[3] }, com);

    // LookupTable setup
    std::string csv_file;
    _Node->get_gait_file(csv_file);
    _LookupTable.readCSV(csv_file);
    _LookupTable.startTrajectory(_Robot->getPosition(), 1.0);

    // Initialise torque control
    _Robot->initTorqueControl();
    LOG(">>> Entered >>>");
}

void RunState::during()
{
    update_controllers();
    update_lookup_table();

    _TorqueOutput = Eigen::Vector4d::Zero();

    // Sum toggled controllers
    if (_Node->get_dev_toggle().external) {
        _TorqueOutput += _CtrlExternal.output();
    }
    if (_Node->get_dev_toggle().friction) {
        _CtrlFriction.loop(_Robot->getVelocity());
        _TorqueOutput += _CtrlFriction.output();
    }
    if (_Node->get_dev_toggle().gravity) {
        _CtrlGravity.loop(_Robot->getPosition());
        _TorqueOutput += _CtrlGravity.output();
    }
    if (_Node->get_dev_toggle().pd && _Node->get_user_command().toggle_walk) {
        _CtrlPD.loop(_LookupTable.getJointPositions(), _Robot->getPosition());
        _TorqueOutput += _CtrlPD.output();
    }
    if (_Node->get_dev_toggle().torque) {
        _CtrlTorque.loop(_Robot->getStrainGauges());
        _TorqueOutput += _CtrlTorque.output();
    }

    // Limit torque output
    for (std::size_t i = 0; i < X2_NUM_JOINTS; i++) {
        if (_TorqueOutput[i] > _Node->get_torque_limit()) {
            _TorqueOutput[i] = _Node->get_torque_limit();
        } else
        if (_TorqueOutput[i] < -_Node->get_torque_limit()) {
            _TorqueOutput[i] = -_Node->get_torque_limit();
        }
    }

    _Robot->setTorque(_TorqueOutput);
    _Node->publish_joint_reference(
        std::vector<double>(
            _LookupTable.getJointPositions().data(),
            _LookupTable.getJointPositions().data() + _LookupTable.getJointPositions().size()
        )
    );
    _Node->publish_joint_state();
    _Node->publish_strain_gauge();
}

void RunState::exit()
{
    LOG("<<< Exited <<<");
}

void RunState::update_controllers()
{
    _CtrlExternal.set_external_torque(
        Eigen::Vector4d(
            _Node->get_external_parameter().torque.cbegin() + 1
        )
    );

    _CtrlFriction.set_static(
        Eigen::Vector4d(
            _Node->get_friction_parameter().static_coefficient.cbegin() + 1
        )
    );
    _CtrlFriction.set_viscous(
        Eigen::Vector4d(
            _Node->get_friction_parameter().viscous_coefficient.cbegin() + 1
        )
    );

    _CtrlPD.set_alphas(
        Eigen::Vector4d(
            _Node->get_pd_parameter().alpha_min.cbegin() + 1
        ),
        Eigen::Vector4d(
            _Node->get_pd_parameter().alpha_max.cbegin() + 1
        )
    );
    Eigen::Matrix4d kp{}, kd{};
    kp.topLeftCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().left_kp.cbegin() + 1);
    kp.bottomRightCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().right_kp.cbegin() + 1);
    kd.topLeftCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().left_kd.cbegin() + 1);
    kd.bottomRightCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().right_kd.cbegin() + 1);
    _CtrlPD.set_gains(kp, kd);

    _CtrlTorque.set_gain(
        Eigen::Vector4d(
            _Node->get_torque_parameter().gain.cbegin() + 1
        )
    );
}

void RunState::update_lookup_table()
{
    // Toggle start and stop trajectory
    _Node->get_user_command().toggle_walk ? _LookupTable.start() : _LookupTable.stop();

    // Set min and max range of motion
    std::array<double, X2_NUM_JOINTS> rom_min, rom_max;
    for (std::size_t i = 0; i < X2_NUM_JOINTS; i++) {
        rom_min[i] = deg2rad(_Node->get_gait_parameter().rom_min[i+1]);
        rom_max[i] = deg2rad(_Node->get_gait_parameter().rom_max[i+1]);
    }
    _LookupTable.setROM(rom_min, rom_max);

    // Set period of gait trajectory
    _LookupTable.setPeriod(_Node->get_gait_parameter().step_period);

    // Increment gait
    _LookupTable.next();
}