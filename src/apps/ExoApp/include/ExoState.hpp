#ifndef __EXO_STATE_HPP__
#define __EXO_STATE_HPP__

/* STL */
#include <string>
#include <iostream>
/* CORC interfaces */
#include "State.h"
#include "X2Robot.h"
#include "ExoNode.hpp"
#include "controller.hpp"
#include "yaml-cpp/yaml.h"
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

private:
    void update_controllers();
    ctrl::AdaptiveController<double, X2_NUM_JOINTS/2, 50>* _CtrlAffcLeftLeg;
    ctrl::AdaptiveController<double, X2_NUM_JOINTS/2, 50>* _CtrlAffcRightLeg;
    ctrl::ExternalController<double, X2_NUM_JOINTS> _CtrlExternal;
    ctrl::PDController<double, X2_NUM_JOINTS> _CtrlPD;
    ctrl::TorqueController<double, X2_NUM_JOINTS> _CtrlTorque;
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