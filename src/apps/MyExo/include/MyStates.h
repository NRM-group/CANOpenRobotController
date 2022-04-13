#ifndef __MY_STATES_H
#define __MY_STATES_H

#include <string>
#include <iostream>
#include <time.h>

#include "MyTrajectory.h"
#include "StateMachine.h"
#include "State.h"
#include "X2Robot.h"

class MyStates : public State
{
public:
    MyStates(StateMachine* sm,
            X2Robot* exo,
            MyTrajectory* traj,
            const char* name = NULL);
    virtual void entry() = 0;
    virtual void during() = 0;
    virtual void exit() = 0;
    

protected:
    X2Robot*        m_robot;
    MyTrajectory*   m_trajectory;

};

#endif//__MY_STATES_H
