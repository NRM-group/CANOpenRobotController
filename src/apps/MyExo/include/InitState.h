#ifndef __INIT_STATE_H
#define __INIT_STATE_H

#include <iostream>

#include "MyStates.h"

class InitState : public MyStates
{
    public:
    InitState(StateMachine* sm,
            X2Robot* exo,
            MyTrajectory* traj,
            const char* name = NULL):
            MyStates (sm, exo, traj, name){};
    void entry(void);
    void during(void);
    void exit(void);
};

#endif//__INIT_STATE_H
