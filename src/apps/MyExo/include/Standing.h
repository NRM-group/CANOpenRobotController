#ifndef __STAND_STATE_H
#define __STAND_STATE_H

#include "MyStates.h"

class Standing : public MyStates 
{
    private:
    double currTrajProgress = 0;
    timespec prevTime;

    public:
    void entry(void);
    void during(void);
    void exit(void);
    Standing(
        StateMachine *m,
        X2Robot *exo,
        MyTrajectory *tg,
        const char *name =NULL) :
    MyStates(m, exo, tg, name){};

};

#endif