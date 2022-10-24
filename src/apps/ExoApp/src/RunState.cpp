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

    // Strain gauge filter parameters
    std::array<double, STRAIN_GAUGE_FILTER_ORDER + 1> coeff;
    _Node->ros_parameter("strain_gauge.coeff_a", buffer);
    std::copy(buffer.begin(), buffer.end(), coeff.begin());
    _Robot->getStrainGaugeFilter().set_coeff_a(coeff);
    _Node->ros_parameter("strain_gauge.coeff_b", buffer);
    std::copy(buffer.begin(), buffer.end(), coeff.begin());
    _Robot->getStrainGaugeFilter().set_coeff_b(coeff);

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

    // AFFC control parameters
    _Node->ros_parameter("affc.left_unknown", left_unknown);
    _Node->ros_parameter("affc.left_learning_rate", left_learning_rate);
    _Node->ros_parameter("affc.left_kp", left_p_gains);
    _Node->ros_parameter("affc.left_kd", left_d_gains);
    _Node->ros_parameter("affc.right_unknown", right_unknown);
    _Node->ros_parameter("affc.right_learning_rate", right_learning_rate);
    _Node->ros_parameter("affc.right_kp", right_p_gains);
    _Node->ros_parameter("affc.right_kd", right_d_gains);
    _Node->ros_parameter("affc.criterions", criterions);
    _Node->ros_parameter("l", lengths);

    _CtrlAffcLeftLeg = new ctrl::AdaptiveController<double, X2_NUM_JOINTS/2, 50>({lengths[0], lengths[1]}, 
                                                                                 left_learning_rate, 
                                                                                 left_p_gains, 
                                                                                 left_d_gains);
    _CtrlAffcRightLeg = new ctrl::AdaptiveController<double, X2_NUM_JOINTS/2, 50>({lengths[2], lengths[3]},
                                                                                  right_learning_rate,
                                                                                  right_p_gains,
                                                                                  right_d_gains);

    // TODO: Add AFFC method to set learned parameters for 
    //      loop() method and tune_loop() method seperately
    _CtrlAffcLeftLeg->set_criterions(deg2rad(criterions[0]), deg2rad(criterions[1]);
    _CtrlAffcLeftLeg->set_inital_guess(left_unknown);

    _CtrlAffcRightLeg->set_criterions(deg2rad(criterions[0]), deg2rad(criterions[1]));
    _CtrlAffcRightLeg->set_inital_guess(right_unknown);

   // butterworth 2nd order fc = 30Hz, fs = 333.333Hz
    _CtrlPositionFilter.set_coeff_a({1.0, -1.2247, 0.4504});
    _CtrlPositionFilter.set_coeff_b({0.0564, 0.1129, 0.0564});
}

    // LookupTable setup
    std::string csv_file;
    _Node->get_gait_file(csv_file);
    _LookupTable.readCSV(csv_file);
    _LookupTable.startTrajectory(_Robot->getPosition(), 1.0);

void RunState::entry()
{
    _Robot->initTorqueControl();
    LOG(">>> Entered >>>");
}

void RunState::during()
{
    update_controllers();
    update_lookup_table();

    _TorqueOutput = Eigen::Vector4d::Zero();
    _ActualPosition = Eigen::Vector4d::Zero();

    _CtrlPositionFilter.filter(_Robot->getPosition());
    _ActualPosition = _CtrlPositionFilter.output();

    // Sum toggled controllers
    if (_Node->get_user_command().toggle_walk) {
        //_LookupTable.next();
        if (_Node->get_dev_toggle().pd) {
            _TorqueOutput += _CtrlPD.output();
        }
    }
    
    
    if (_Node->get_dev_toggle().external) {
        _TorqueOutput += _CtrlExternal.output();
    }
    // TODO: Add mass and coriolis dev toggles for AFFC to dev_toggle.msg
    if (_Node>get_dev_toggle().mass) {
        _CtrlAffcLeftLeg->loop(, ctrl::MASS_COMP);
        _CtrlAffcRightLeg->loop(, ctrl::MASS_COMP);

        Eigen::Vector4d friction_torque;
        friction_torque << _CtrlAffcLeftLeg->output(), _CtrlAffcRightLeg->output();

        _TorqueOutput += friction_torque;
    }
    if (_Node->get_dev_toggle().coriolis) {
        _CtrlAffcLeftLeg->loop(, ctrl::CORIOLIS_COMP);
        _CtrlAffcRightLeg->loop(, ctrl::CORIOLIS_COMP);

        Eigen::Vector4d friction_torque;
        friction_torque << _CtrlAffcLeftLeg->output(), _CtrlAffcRightLeg->output();

        _TorqueOutput += friction_torque;
    }
    // TODO: Add mass and coriolis dev toggles for AFFC to dev_toggle.msg
    if (_Node>get_dev_toggle().mass) {
        _CtrlAffcLeftLeg->loop(, ctrl::MASS_COMP);
        _CtrlAffcRightLeg->loop(, ctrl::MASS_COMP);

        Eigen::Vector4d friction_torque;
        friction_torque << _CtrlAffcLeftLeg->output(), _CtrlAffcRightLeg->output();

        _TorqueOutput += friction_torque;
    }
    if (_Node->get_dev_toggle().coriolis) {
        _CtrlAffcLeftLeg->loop(, ctrl::CORIOLIS_COMP);
        _CtrlAffcRightLeg->loop(, ctrl::CORIOLIS_COMP);

        Eigen::Vector4d friction_torque;
        friction_torque << _CtrlAffcLeftLeg->output(), _CtrlAffcRightLeg->output();

        _TorqueOutput += friction_torque;
    }
    if (_Node->get_dev_toggle().friction) {
        _CtrlAffcLeftLeg->loop(, ctrl::FRICTION_COMP);
        _CtrlAffcRightLeg->loop(, ctrl::FRICTION_COMP);

        Eigen::Vector4d friction_torque;
        friction_torque << _CtrlAffcLeftLeg->output(), _CtrlAffcRightLeg->output();

        _TorqueOutput += friction_torque;
    }
    if (_Node->get_dev_toggle().gravity) {
        _CtrlAffcLeftLeg->loop(, ctrl::GRAVITY_COMP);
        _CtrlAffcRightLeg->loop(, ctrl::GRAVITY_COMP);

        Eigen::Vector4d gravity_torque;
        gravity_torque << _CtrlAffcLeftLeg->output(), _CtrlAffcRightLeg->output();

        _TorqueOutput += gravity_torque;
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