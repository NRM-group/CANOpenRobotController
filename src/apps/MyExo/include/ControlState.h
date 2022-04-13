#ifndef __CONTROLSTATE_H_INCLUDED
#define __CONTROLSTATE_H_INCLUDED

#include "MyStates.h"
#include "controller.h"

class ControlState : public MyStates
{
    public:
    void entry(void);
    void during(void);
    void exit(void);
    ControlState(StateMachine* m,
            X2Robot* exo,
            MyTrajectory* tg,
            const char* name=NULL) : MyStates(m, exo, tg, name) {};
};

#endif