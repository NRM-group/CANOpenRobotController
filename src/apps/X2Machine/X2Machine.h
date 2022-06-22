/**
 * @file X2Machine.h
 * @author nilp amin (nilpamin2@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef SRC_X2MACHINE_H
#define SRC_X2MACHINE_H

#include <sys/time.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/init_options.hpp"

#include "X2Robot.h"
#include "StateMachine.h"

#include "X2MachineROS2.h"
#include "X2FollowerState.h"

class X2Machine : public StateMachine {

public:
    X2Machine(int argc, char* argv[], const float updateT);

    X2MachineROS2* x2MachineRos2_;
    X2Robot* robot_;

    bool running = false;

    void init(void);
    void end(void);
    
    void update(void);
    void hwStateUpdate(void);
    void initRobot(X2Robot* robot);
    bool configureMasterPDOs(void);

    std::string getGaitCycle(void);
    X2FollowerState* x2FollowerState_;


private:
    EventObject(StartExo)* startExo;

    std::string robotName_;

    std::chrono::steady_clock::time_point time0;
    double time;

    std::shared_ptr<rclcpp::Node> node_;
};

#endif