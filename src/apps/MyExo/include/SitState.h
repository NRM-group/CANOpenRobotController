#ifndef __SIT_STATE_H
#define __SIT_STATE_H
#include <iostream>

#include "MyStates.h"

/**
 * @brief State for the exo representing when it is sitting down
 * 
 */
class SitState : public MyStates 
{
    public:
    SitState(StateMachine* m,
            X2Robot* exo,
            MyTrajectory* traj,
            const char* name = NULL)
    : MyStates(m, exo, traj, name) {}; 
    void entry(void);
    void during(void);
    void exit(void);
};

#endif