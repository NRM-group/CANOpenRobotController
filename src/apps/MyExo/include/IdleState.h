#ifndef __IDLE_H_INCLUDED
#define __IDLE_H_INCLUDED

#include "MyStates.h"

/**
 * @brief State for reading the joints 
 * The system is loose, but the joints can be read
 * 
 */

class IdleState : public MyStates
{
    /** State initialisation for the idle state
    */
   public:
   void entry(void);
   void during(void);
   void exit(void);
   IdleState(StateMachine *m,
            X2Robot *exo,
            MyTrajectory *tg,
            const char *name = NULL) :
    MyStates(m, exo, tg, name) {};
};

#endif
