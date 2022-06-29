#ifndef SRC_X2SAFETYSTATE_H
#define SRC_X2SAFETYSTATE_H

#include "State.h"
#include "X2Robot.h"

class X2SafetyState : public State {
    X2Robot* robot_;
public:
    void entry(void);
    void during(void);
    void exit(void);

    X2SafetyState(StateMachine *m, X2Robot* exo, const float updateT, const char* name = NULL);
private:
    const int freq_;
    Eigen::VectorXd stationaryVelocity;
};

#endif