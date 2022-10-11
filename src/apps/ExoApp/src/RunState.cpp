#include "ExoState.hpp"
#define LOG(x)  spdlog::info("[RunState]: {}", x)

RunState::RunState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Run State"), _Robot(robot), _Node(node), _CtrlExternal{},
    , _CtrlPD{}, _CtrlTorque{}
{
    std::vector<double> left_unknown, left_learning_rate, left_p_gains, left_d_gains;
    std::vector<double> right_unknown, right_learning_rate, right_p_gains, right_d_gains;
    std::vector<double> criterions, lengths;

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