#include "X2Robot.h"

void X2Robot::init()
{
    _PosMotorProfile = { 4000000, 240000, 240000 };
    _VelMotorProfile = { 0, 240000, 240000 };
    _ControlMode = ControlMode::CM_UNCONFIGURED;
    initialiseInputs();
    initialiseJoints();
    initialiseNetwork();
}

void X2Robot::kill_robot()
{
    spdlog::info("Shutting down...");
    this->disable();
    exit(EXIT_SUCCESS);
}

void X2Robot::update_robot()
{
    this->updateRobot();
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        if (std::abs(jointTorques_[i]) > _MaxTorque) {
            spdlog::error(
                "X2Robot: JOINT {} exceeded maximum TORQUE {} with {}",
                i, _MaxTorque, jointTorques_[i]
            );
            kill_robot();
        }
        if (std::abs(jointVelocities_[i]) > _MaxVelocity) {
            spdlog::error(
                "X2Robot: JOINT {} exceeded maximum VELOCITY {} with {}",
                i, _MaxVelocity, jointVelocities_[i]
            );
            kill_robot();
        }
        if (i % 2) { // knee
            if (jointPositions_[i] < _MinKnee) {
                spdlog::error(
                    "X2Robot: JOINT {} exceeded minimum POSITION {} with {}",
                    i, _MinKnee, jointPositions_[i]
                );
                kill_robot();
            } else if (jointPositions_[i] > _MaxKnee) {
                spdlog::error(
                    "X2Robot: JOINT {} exceeded maximum POSITION {} with {}",
                    i, _MaxKnee, jointPositions_[i]
                );
                kill_robot();
            }
        } else { // hip
            if (jointPositions_[i] < _MinHip) {
                spdlog::error(
                    "X2Robot: JOINT {} exceeded minimum POSITION {} with {}",
                    i, _MinHip, jointPositions_[i]
                );
                kill_robot();
            } else if (jointPositions_[i] > _MaxHip) {
                spdlog::error(
                    "X2Robot: JOINT {} exceeded maximum POSITION {} with {}",
                    i, _MaxHip, jointPositions_[i]
                );
                kill_robot();
            }
        }
        _StrainGauge[i] = _ForceSensors[i]->getForce();
    }
}

void X2Robot::set_position(const Eigen::Vector4d &pos)
{
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        auto retval = joints[i]->setPosition(pos[i]);
        if (retval == INCORRECT_MODE) {
            spdlog::error("JOINT {} not in POSITION mode", i);
        } else if (retval != SUCCESS) {
            spdlog::error(
                "X2Robot: Unexpected behaviour for set_position({}) at JOINT {}",
                pos[i], i
            );
        }
    }
}

void X2Robot::set_velocity(const Eigen::Vector4d &vel)
{
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        auto retval = joints[i]->setVelocity(vel[i]);
        if (retval == INCORRECT_MODE) {
            spdlog::error("JOINT {} not in VELOCITY mode", i);
        } else if (retval != SUCCESS) {
            spdlog::error(
                "X2Robot: Unexpected behaviour for set_velocity({}) at JOINT {}",
                vel[i], i
            );
        }
    }
}

void X2Robot::set_torque(const Eigen::Vector4d &tor)
{
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        auto retval = joints[i]->setTorque(tor[i]);
        if (retval == INCORRECT_MODE) {
            spdlog::error("JOINT {} not in TORQUE mode", i);
        } else if (retval != SUCCESS) {
            spdlog::error(
                "X2Robot: Unexpected behaviour for set_torque({}) at JOINT {}",
                tor[i], i
            );
        }
    }
}

bool X2Robot::set_control(ControlMode mode)
{
    bool success = true;

    for (auto &j : joints) {
        if (j->setMode(mode, _PosMotorProfile) != mode) {
            spdlog::error("X2Robot: Unexpected behaviour for set_control({})", mode);
            success = false;
        }
        j->readyToSwitchOn();
    }

    usleep(10000);

    for (auto &j : joints)
        j->enable();

    if (success) { _ControlMode = mode; }
    return success;
}

bool X2Robot::initialiseInputs()
{
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        _ForceSensors[i] = std::make_shared<FourierForceSensor>(i + 17);
        inputs.push_back(_ForceSensors[i].get());
    }
    spdlog::info("X2Robot: Initialised inputs");
    return true;
}

bool X2Robot::initialiseJoints()
{
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        _MotorDrives[i] = std::make_shared<Drive>(new CopleyDrive(i + 1));
        if (i % 2) { // knees
            joints.push_back(new X2Joint(i, _MinKnee, _MaxKnee, KneeJDP, _MotorDrives[i].get()));
        } else { // hips
            joints.push_back(new X2Joint(i, _MinHip, _MaxHip, HipJDP, _MotorDrives[i].get()));
        }
    }
    spdlog::info("X2Robot: Initialised joints");
    return true;
}

bool X2Robot::initialiseNetwork()
{
    for (auto &j : joints) {
        if (j->initNetwork()) {
            spdlog::error("X2Robot: Failed to initialise network");
        }
    }
    spdlog::info("X2Robot: Initialised network");
    return true;
}