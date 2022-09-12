#include "X2Robot.h"

void X2Robot::start_trajectory()
{
    spdlog::info("X2Robot: Starting trajectory...");
    this->update_robot();
    sleep(1);
    this->update_robot();
    _LookupTable.startTrajectory(jointPositions_, 4);
}

void X2Robot::execute()
{
    _TorqueOutput = Eigen::Vector4d::Zero();
    _StrainGauge = _StrainGaugeScale.cwiseProduct(_StrainGauge - _StrainGaugeOffset);

    // Loop toggled controllers
    if (_UserCommand.toggle_walk) {
        _LookupTable.next();
        if (_DevToggle.pd) {
            _CtrlPD.loop(_LookupTable.getJointPositions() - jointPositions_);
            _TorqueOutput += _CtrlPD.output();
        }
    }
    if (_DevToggle.external) {
        _TorqueOutput += _CtrlExternal.output();
    }
    if (_DevToggle.friction) {
        _CtrlFriction.loop(jointVelocities_);
        _TorqueOutput += _CtrlFriction.output();
    }
    if (_DevToggle.gravity) {
        _CtrlGravity.loop(jointPositions_);
        _TorqueOutput += _CtrlGravity.output();
    }
    if (_DevToggle.torque) {
        _CtrlButterStrainGauge.filter(_StrainGauge);
        _CtrlTorque.loop(_CtrlButterStrainGauge.output());
        _TorqueOutput += _CtrlTorque.output();
    }

    // Limit torque output
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        if (_TorqueOutput[i] > _TorqueLimit) {
            _TorqueOutput[i] = _TorqueLimit;
        } else if (_TorqueOutput[i] < -_TorqueLimit) {
            _TorqueOutput[i] = -_TorqueLimit;
        }
    }

    this->set_torque(_TorqueOutput);
}

void X2Robot::calibrate()
{
    spdlog::info("X2Robot: Calibrating...");
    this->update_robot();
    sleep(1);
    this->update_robot();
    _StrainGaugeOffset = _StrainGauge;
    spdlog::info("X2Robot: Strain Gauge Offset [{}, {}, {}, {}]",
        _StrainGaugeOffset[0],
        _StrainGaugeOffset[1],
        _StrainGaugeOffset[2],
        _StrainGaugeOffset[3]
    );
}