#include "ExoState.hpp"
#define LOG(x)  spdlog::info("[RunState]: {}", x)

RunState::RunState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Run State"), _Robot(robot), _Node(node), _CtrlExternal{},
    _CtrlFriction{}, _CtrlGravity{}, _CtrlPD{}, _CtrlTorque{}
{
    // Strain gauge filter parameters
    std::vector<double> coeff_a, coeff_b;
    std::array<double, STRAIN_GAUGE_FILTER_ORDER + 1> coeff;
    _Node->ros_parameter("strain_gauge.coeff_a", coeff_a);
    _Node->ros_parameter("strain_gauge.coeff_b", coeff_b);
    std::copy(coeff_a.begin(), coeff_a.end(), coeff.begin());
    _Robot->getStrainGauges().set_coeff_a(coeff);
    std::copy(coeff_b.begin(), coeff_b.end(), coeff.begin());
    _Robot->getStrainGauges().set_coeff_b(coeff);

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

    Eigen::Vector4d upperVelLimit { 0.1, 0.1, 0.1, 0.1 };
    Eigen::Vector4d lowerVelLimit { -0.1, -0.1, -0.1, -0.1 };
    _CtrlFriction.set_deadband(lowerVelLimit, upperVelLimit);
}

void RunState::entry()
{
    _Robot->initTorqueControl();
    LOG(">>> Entered >>>");
}

void RunState::during()
{
    update_controllers();
    // FIXME:
    _TorqueOutput = Eigen::Vector4d::Zero();
    //_StrainGauge = _StrainGaugeScale.cwiseProduct(_StrainGauge - _StrainGaugeOffset);
    //_CtrlPD.loop(_LookupTable.getJointPositions() - jointPositions_);
    _CtrlFriction.loop(_Robot->getVelocity());
    _CtrlGravity.loop(_Robot->getPosition());
    //_CtrlButterStrainGauge.filter(_StrainGauge);
    //_CtrlTorque.loop(_CtrlButterStrainGauge.output());
    // spdlog::info("POS: {}", _Robot->getPosition()[0]);

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
    if (_Node->get_dev_toggle().friction) {
        _TorqueOutput += _CtrlFriction.output();
    }
    if (_Node->get_dev_toggle().gravity) {
        _TorqueOutput += _CtrlGravity.output();
    }
    if (_Node->get_dev_toggle().torque) {
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
    // spdlog::info("TOR: {}", _TorqueOutput[0]);
    _Robot->setTorque(_TorqueOutput);
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
    kp.bottomRightCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().left_kd.cbegin() + 1);
    kd.topLeftCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().right_kp.cbegin() + 1);
    kd.bottomRightCorner(2, 2) = Eigen::Matrix2d(_Node->get_pd_parameter().right_kd.cbegin() + 1);
    _CtrlPD.set_gains(kp, kd);
    // TODO:
    //_CtrlTorque.set_gain()
}