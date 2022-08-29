#include "X2Robot.h"

X2Robot::X2Robot()
    : Robot(X2_NUM_JOINTS), _StrainGauge{},
    _PosMotorProfile{ 4000000, 240000, 240000 },
    _VelMotorProfile{ 0, 240000, 240000 },
    _ControlMode(ControlMode::CM_UNCONFIGURED)
{
    initialiseInputs();
    initialiseJoints();
    initialiseNetwork();
    spdlog::info("X2Robot: Ready");
}

X2Robot::~X2Robot()
{
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        delete inputs[i];
        delete joints[i];
    }
    spdlog::info("X2Robot: Destructed");
}

void X2Robot::update()
{
    this->updateRobot();
    for (unsigned i = 0; i < X2_NUM_JOINTS; i++) {
        if (std::abs(jointTorques_[i]) > _MaxTorque) {
            spdlog::error(
                "X2Robot: JOINT {} exceeded maximum TORQUE {} at {}",
                i, _MaxTorque, jointTorques_[i]
            );
            this->disable();
        }
        if (std::abs(jointVelocities_[i]) > _MaxVelocity) {
            spdlog::error(
                "X2Robot: JOINT {} exceeded maximum VELOCITY {} at {}",
                i, _MaxVelocity, jointVelocities_[i]
            );
            this->disable();
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
        auto retval = joints[i]->setPosition(vel[i]);
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
        auto retval = joints[i]->setPosition(tor[i]);
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

const Eigen::Vector4d &X2Robot::get_strain_gauge()
{
    return _StrainGauge;
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

bool X2Robot::loadParametersFromYAML(YAML::Node params)
{
    spdlog::info("X2Robot: Loading safety parameters...");
    auto config = params["x2"]["ros__parameters"];
    _MaxHip = config["hip_max"].as<double>();
    _MinHip = config["hip_min"].as<double>();
    _MaxKnee = config["knee_max"].as<double>();
    _MinKnee = config["knee_min"].as<double>();
    _MaxTorque = config["max_torque"].as<double>();
    _MaxVelocity = config["max_velocity"].as<double>();
    spdlog::info("X2Robot: Max hip: {}", _MaxHip);
    spdlog::info("X2Robot: Min hip: {}", _MinHip);
    spdlog::info("X2Robot: Max knee: {}", _MaxKnee);
    spdlog::info("X2Robot: Min knee: {}", _MinKnee);
    spdlog::info("X2Robot: Max torque: {}", _MaxTorque);
    spdlog::info("X2Robot: Max velocity: {}", _MaxVelocity);
}