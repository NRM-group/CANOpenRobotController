#ifndef __STANDING_H_INCLUDED
#define __STANDING_H_INCLUDED

#include <iostream>
#include "MyStates.h"

/**
 * @brief Class taht contains the Exo's standing position
 * Statitonary and waiting to act
 * 
 */

class StandState : public MyStates 
{
    public:
    StandState(StateMachine* m,
                X2Robot* exo,
                MyTrajectory* traj,
                const char* name = NULL):
    MyStates(m, exo, traj, name) {};
    void entry(void);
    void during(void);
    void exit(void);
    

};

#endif
