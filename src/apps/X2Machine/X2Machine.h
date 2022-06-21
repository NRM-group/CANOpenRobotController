/**
 * @file X2.h
 * @author Nilp Amin (nilpamin2@gmail.com.au)
 * @brief  Implements a simple ROS based statemachine from controlling the X2 with custom controllers.
 * @version 0.1
 * @date 2022-06-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef SRC_X2MACHINE_H
#define SRC_X2MACHINE_H

#include <sys/time.h>
#include <string>

#include "X2Robot.h"
#include "StateMachine.h"

#include "X2MachineROS.h"

class X2Machine : public StateMachine {

public:
    X2Machine(int argc, char* argv[], const float updateT);

    X2MachineROS* x2MachineRos_;
    X2Robot* robot_;

    bool running = false;

    void init();
    void end();
    
    void update();
    void hwStateUpdate();
    void initRobot(X2Robot* robot);
    bool configureMasterPDOs();

    std::string getGaitCycle(void);
    X2FollowerState* x2FollowerState_;


private:
    EventObject(StartExo)* startExo;

    std::string robotName_;

    std::chrono::steady_clock::time_point time0;
    double time;

};

#endif