#include "MyStates.h"

MyStates::MyStates(StateMachine* sm,
        X2Robot* exo,
        MyTrajectory* traj,
        const char* name) :
                State(sm, name),
                m_robot(exo),
                m_trajectory(traj)
{
}
