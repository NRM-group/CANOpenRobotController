#ifndef X2ROBOT_H
#define X2ROBOT_H

#include <map>
#include <chrono>

#include <Eigen/Dense>

#include "Robot.h"
#include "X2Joint.h"
#include "Keyboard.h"
#include "CopleyDrive.h"
#include "FourierForceSensor.h"

#define X2_NUM_JOINTS 4

JointDrivePairs HipJDP
{
    250880,
    0,
    1.57,
    0
};

JointDrivePairs KneeJDP
{
    0,
    250880,
    0,
    -1.57
};

class X2Robot : public Robot
{
public:
    X2Robot();
    ~X2Robot();

public:
    void update();
    void set_position(const Eigen::Vector4d &pos);
    void set_velocity(const Eigen::Vector4d &vel);
    void set_torque(const Eigen::Vector4d &tor);
    bool set_control(ControlMode mode);
    const Eigen::Vector4d &get_strain_gauge();

private:
    bool initialiseInputs() override;
    bool initialiseJoints() override;
    bool initialiseNetwork() override;
    bool loadParametersFromYAML(YAML::Node params) override;

private:
    motorProfile _PosMotorProfile;
    motorProfile _VelMotorProfile;
    // RobotParameters _Parameters;
    ControlMode _ControlMode;
    std::array<std::shared_ptr<Drive>, X2_NUM_JOINTS> _MotorDrives;
    std::array<std::shared_ptr<FourierForceSensor>, X2_NUM_JOINTS> _ForceSensors;
    double _MaxTorque, _MaxVelocity, _MaxHip, _MinHip, _MaxKnee, _MinKnee;
    Eigen::Vector4d _StrainGauge;
};

#endif