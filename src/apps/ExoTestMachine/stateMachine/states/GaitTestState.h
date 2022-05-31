#ifndef GATITESTSTATE_H_INCLUDED
#define GAITTESTSTATE_H_INCLUDED
#include "ExoTestState.h"
#include "LookupTable.h"

#define PERIOD 100 //Period in ms

class GaitTestState : public ExoTestState {
    private:
        LookupTable csvReader; //Csv interpreter for the gait pattern
        timespec prevTime; //Used to keep track of how fast the leg moves
        double gaitIndex;

    public:
        void entry(void);
        void during(void);
        void exit(void);
        GaitTestState(StateMachine *m, X2Robot *exo, DummyTrajectoryGenerator *tg, const char *name = NULL) : ExoTestState(m, exo, tg, name){};
};

#endif