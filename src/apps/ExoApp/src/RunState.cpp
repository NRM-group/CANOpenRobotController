#include "ExoState.hpp"
#define LOG(x)      spdlog::info("[RunState]: {}", x)

RunState::RunState(const std::shared_ptr<X2Robot> robot,
                   const std::shared_ptr<ExoNode> node)
    : State("Run State"), _Robot(robot), _Node(node), _GaitTracjectory{}
{
    _Node->ros_declare(
        {
            "l", "m", "s",
            "affc.left_unknown",
            "affc.learning_rate",
            "affc.kp",
            "affc.kd",
            "affc.right_unknown",
            "affc.criterions"
        }
    );

    _ActualPosition = Eigen::Vector4d::Zero();
    _DesiredPosition = Eigen::Vector4d::Zero();
    _DesiredVelocity = Eigen::Vector4d::Zero();
    _DesiredAccel = Eigen::Vector4d::Zero();
}

void RunState::entry()
{
    std::vector<double> learning_rate, p_gains, d_gains;
    std::vector<double> left_unknown, right_unknown;
    std::vector<double> criterions, lengths;

    // AFFC control parameters
    _Node->ros_parameter("l", lengths);
    _Node->ros_parameter("affc.learning_rate", learning_rate);
    _Node->ros_parameter("affc.kp", p_gains);
    _Node->ros_parameter("affc.kd", d_gains);
    _Node->ros_parameter("affc.left_unknown", left_unknown);
    _Node->ros_parameter("affc.right_unknown", right_unknown);
    _Node->ros_parameter("affc.criterions", criterions);

    _CtrlAffc = new ctrl::AdaptiveController<double, X2_NUM_JOINTS, 50>(lengths, learning_rate, p_gains, d_gains);

    _CtrlAffc->set_criterions(deg2rad(criterions[0]), deg2rad(criterions[1]));
    _CtrlAffc->set_inital_guess(left_unknown, right_unknown);

    // butterworth 2nd order fc = 30Hz, fs = 333.333Hz
    _CtrlPositionFilter.set_coeff_a({1.0, -1.2247, 0.4504});
    _CtrlPositionFilter.set_coeff_b({0.0564, 0.1129, 0.0564});

    // initalise time0 for gait trajectory
    _Time0 = std::chrono::steady_clock::now();

    // initalise torque controller
    _Robot->initTorqueControl();
    LOG(">>> Entered >>>");
}

void RunState::during()
{
    _TorqueOutput = Eigen::Vector4d::Zero();

    // filter current joint positions to remove noise
    _CtrlPositionFilter.filter(_Robot->getPosition());
    _ActualPosition << _CtrlPositionFilter.output();

    // obtain delta in time since last loop
    double time  = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - _Time0).count()/1.0e9;

    // update current dynamics from ref trajectory
    _DesiredPosition << _GaitTracjectory.getPosition(time);
    _DesiredVelocity << _GaitTracjectory.getVelocity(time);
    _DesiredAccel << _GaitTracjectory.getAccelaration(time);

    // check if the AFFC algorithm is finished
    if (_CtrlAffc->is_finished(ctrl::LEFT_LEG) && _CtrlAffc->is_finished(ctrl::RIGHT_LEG)) {
        spdlog::info("AFFC is finished for both legs.");
        return;
    }

    // iterate AFFC tune algorithm once
    if (_PeriodCounter * 5.0 < time) {
        //TODO: Add the code for this
    } else {
        // TODO: Make sure to only use the first 2 elements for left leg and last 2 elements for right leg...otherwise garbage will come out
        _CtrlAffc->tune_loop(_DesiredPosition, _ActualPosition, _DesiredVelocity, _DesiredAccel, false, ctrl::LEFT_LEG);
        _TorqueOutput += _CtrlAffc->output();

        _CtrlAffc->tune_loop(_DesiredPosition, _ActualPosition, _DesiredVelocity, _DesiredAccel, false, ctrl::RIGHT_LEG);
        _TorqueOutput += _CtrlAffc->output();
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