#ifndef __SITTING_H_INCLUDED
#define __SITTING_H_INCLUDED

#include "MyStates.h"

/**
 * @brief State for the practice App
 * Transitions the Exo into the sitting state
 */
class Sitting : public MyStates
{
    /*Parameters for tracking the progression of the
    * trajectory
    */
    
    private:
    double currTrajProgress = 0;
    timespec prevTime;
 
    public:
    /**
     * @brief Set up the robot and Trajectory Generatror objects
     * 
     */
    void entry(void);
    void during(void);
    void exit(void);
    Sitting(StateMachine *m, 
            X2Robot * exo,
            MyTrajectory *tg, 
            const char *name = NULL) : MyStates(m, exo, tg, name) {};
};
#endif