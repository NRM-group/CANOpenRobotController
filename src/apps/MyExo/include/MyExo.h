#ifndef __MY_EXO_H
#define __MY_EXO_H 

#include "MyTrajectory.h"
#include "StateMachine.h"
#include "X2Robot.h"
#include "InitState.h"
#include "SitState.h"
#include "Sitting.h"
#include "Standing.h"
#include "StandState.h"
#include "ControlState.h"
#include "IdleState.h"

#define OWNER ((MyExo*) owner)

class MyExo : public StateMachine
{
public:
    MyExo(void);
    void init(void);
    void end(void);
    void update(void);
    void hwStateUpdate(void);
    void configureMasterPDOs(void);

    State* gettCurState(void);
    void initRobot(X2Robot* robot);
public:
    //State pointers 
    InitState* initState;
    SitState* sitState;
    Sitting* sittingState;
    Standing* standingState;
    StandState* standState;
    ControlState* controlState;
    IdleState* idleState; //The new initial step


public:
    bool                                    running = false;
    std::chrono::steady_clock::time_point   time0;
    double                                  time;
    bool                                    traj_complete;
    MyTrajectory*                           traj;

protected:
    X2Robot*                                m_robot;
    LogHelper                               m_logger;

private:
    EventObject(Start)*                     m_start_exo;
    EventObject(MoveSit)*                   m_move_sit;
    EventObject(MoveStand)*                 m_move_stand;
    EventObject(Pause)*                     m_pause;
    EventObject(EndTraj)*                   m_endTraj;
    EventObject(ExoCal)*                    m_calibrate;
    EventObject(MoveControl)*               m_move_control;
    EventObject(MoveIdle)*                  m_move_idle;
};

#endif//__MY_EXO_H
