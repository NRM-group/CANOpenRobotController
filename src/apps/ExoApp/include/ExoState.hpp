#ifndef __EXO_STATE_HPP__
#define __EXO_STATE_HPP__

/**************** RATE LIMITS ****************/
#define INITIAL_GAIT_RATE   0.2   // rad/s
#define INITIAL_STAND_RATE  0.08  // rad/s default: 0.15
#define POSITION_RATE       0.004 // rad/s * T
#define MIN_ROM_RATE        0.003 // rad/s * T
#define MAX_ROM_RATE        0.003 // rad/s * T
/*(*******************************************/

/* STL */
#include <string>
#include <iostream>
/* CORC interfaces */
#include "State.h"
#include "X2Robot.h"
#include "ExoNode.hpp"
#include "controller.hpp"
#include "LookupTable.hpp"
#include "yaml-cpp/yaml.h"
//#include "pd.hpp"
/* Standard interfaces */
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
/* Exo interfaces */
#include "exo_msgs/msg/dev_toggle.hpp"
#include "exo_msgs/msg/endpoint.hpp"
#include "exo_msgs/msg/external_parameter.hpp"
#include "exo_msgs/msg/friction_parameter.hpp"
#include "exo_msgs/msg/gait_parameter.hpp"
#include "exo_msgs/msg/patient_parameter.hpp"
#include "exo_msgs/msg/pd_parameter.hpp"
#include "exo_msgs/msg/sit_to_stand_parameter.hpp"
#include "exo_msgs/msg/user_command.hpp"
/* ROS interface namespaces */
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace exo_msgs::msg;

/*************
 * OFF STATE *
 *************/
class OffState : public State
{
public:
    OffState(const std::shared_ptr<X2Robot> robot,
             const std::shared_ptr<ExoNode> node);

    void entry() override;
    void during() override;
    void exit() override;

private:
    const std::shared_ptr<X2Robot> _Robot;
    const std::shared_ptr<ExoNode> _Node;
    std::size_t _Counter;
};

/*************
 * RUN STATE *
 *************/
class RunState : public State
{
public:
    RunState(const std::shared_ptr<X2Robot> robot,
             const std::shared_ptr<ExoNode> node);

    void entry() override;
    void during() override;
    void exit() override;

private:
    const std::shared_ptr<X2Robot> _Robot;
    const std::shared_ptr<ExoNode> _Node;
    Eigen::Vector4d _TorqueOutput;
    Eigen::Vector4d _LastTorqueOutput;
    Eigen::Vector4d _GravityComp;
    Eigen::Vector4d _ActualPosition;
    Eigen::Vector4d _DesiredPosition;
    Eigen::Vector4d _DesiredVelocity;
    Eigen::Vector4d _DesiredAccel;
    static void rate_limit(const Eigen::Vector4d &target, Eigen::Vector4d &current, double rate);

private:
    bool _DuringGait;
    Eigen::Vector4d _Position;
    Eigen::Vector4d _MinROM;
    Eigen::Vector4d _MaxROM;

private:
    void update_controllers();
    ctrl::AdaptiveController<double, X2_NUM_JOINTS, 25>* _CtrlAffc;
    ctrl::ExternalController<double, X2_NUM_JOINTS> _CtrlExternal;
    ctrl::FrictionController<double, X2_NUM_JOINTS> _CtrlFriction;
    ctrl::GravityController<double, X2_NUM_JOINTS> _CtrlGravity;
    ctrl::PDController<double, X2_NUM_JOINTS> _CtrlPD;
    ctrl::TorqueController<double, X2_NUM_JOINTS> _CtrlTorque;
    ctrl::TransparentWalkController<double, X2_NUM_JOINTS> _CtrlTransparentWalk;
    ctrl::Butterworth<double, X2_NUM_JOINTS, 2> _CtrlPositionFilter;

private:
    void update_lookup_table();
    LookupTable<double, X2_NUM_JOINTS> _LookupTable;
};

/*************
 * SET STATE *
 *************/
class SetState : public State
{
public:
    SetState(const std::shared_ptr<X2Robot> robot,
             const std::shared_ptr<ExoNode> node);

    void entry() override;
    void during() override;
    void exit() override;

private:
    const std::shared_ptr<X2Robot> _Robot;
    const std::shared_ptr<ExoNode> _Node;
};

#endif//__EXO_STATE_HPP__