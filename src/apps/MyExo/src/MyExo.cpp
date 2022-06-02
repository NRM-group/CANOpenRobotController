#include "MyExo.h"

MyExo::MyExo(void)
{
    traj = new MyTrajectory(X2_NUM_JOINTS);
    m_robot = new X2Robot();

    //Events
    m_start_exo = new Start(this);
    m_move_sit = new MoveSit(this);
    m_move_stand = new MoveStand(this);
    m_pause = new Pause(this);
    m_endTraj = new EndTraj(this);
    m_calibrate = new ExoCal(this);
    m_move_control = new MoveControl(this);
    m_move_idle = new MoveIdle(this);

    //States
    idleState = new IdleState(this, m_robot, traj);
    initState = new InitState(this, m_robot, traj);
    sitState = new SitState(this, m_robot, traj);
    standState = new StandState(this, m_robot, traj);
    sittingState = new Sitting(this, m_robot, traj);
    standingState = new Standing(this, m_robot, traj);
    controlState = new ControlState(this, m_robot, traj);

    //Transitions
    NewTransition(initState, m_move_idle, idleState);
    NewTransition(idleState, m_move_sit, sittingState);
    NewTransition(idleState, m_move_stand, standingState);
    NewTransition(initState, m_calibrate, sittingState);
    
    NewTransition(sittingState, m_endTraj, sitState);
    NewTransition(sitState, m_move_stand, standingState);
    NewTransition(sitState, m_move_idle, idleState);

    NewTransition(standingState, m_endTraj, standState);
    NewTransition(standState, m_move_sit, sittingState);
    NewTransition(standState, m_move_control, controlState);
    NewTransition(standState, m_move_idle, idleState);
    
    NewTransition(controlState, m_move_stand, standingState);
    NewTransition(initState, m_start_exo, idleState);
    //TODO:
    //Finish transitions
    StateMachine::initialize(initState);
}


void MyExo::init()
{
    spdlog::debug("Initialise Practice Program");
    initialised = m_robot->initialise();
    running = true;

    //initilised the data logger
    time0 = std::chrono::steady_clock::now();
    
    m_logger.initLogger("test_logger", "logs/testLog.csv", LogFormat::CSV, true);
    m_logger.add(time, "time");
    m_logger.add(m_robot->getPosition(), "JointPositions");
    m_logger.startLogger();
}


void MyExo::end() 
{
    spdlog::debug("Ending the Practice Program");
    m_logger.endLog();
    delete m_robot;
}


//TRANSITION EVENTS

bool MyExo::Start::check(void) 
{
    if (OWNER->m_robot->keyboard->getS() == true) {
        return true;
    }
    return false;
}

bool MyExo::EndTraj::check(void) 
{
    return OWNER->traj->is_trajectory_finished();
}

bool MyExo::MoveSit::check(void)
{
    if (OWNER->m_robot->keyboard->getS() == true) {
        return true;
    }
    return false;

}

bool MyExo::MoveIdle::check(void) 
{
    if (OWNER->m_robot->keyboard->getD() == true) {
        return true;
    }
    return false;
}


bool MyExo::MoveStand::check(void) 
{
    if (OWNER->m_robot->keyboard->getW() == true) {
        return true;
    }
    return false;
}

bool MyExo::ExoCal::check(void)
{
    if (OWNER->m_robot->keyboard->getA() == true) {
        #ifndef NOROBOT
            spdlog::info("Begin Robot Calibration");
            spdlog::info("Homing");

            OWNER->m_robot->disable();
            OWNER->m_robot->homing();
            spdlog::info("Calibration Complete");
        #endif
        return true;
    }
    return false;
}

bool MyExo::MoveControl::check(void)
{
    if (OWNER->m_robot->keyboard->getX() == true) {
        return true;
    }
    return false;
}

bool MyExo::Pause::check(void) {
    return true;
}
/**
 * @brief State machine to hardware interface method
 * The following steps are neccesary to integrate the SM with the 
 * hardware
 * 
 */

void MyExo::hwStateUpdate(void) 
{
    m_robot->updateRobot();
}

void MyExo::configureMasterPDOs() 
{
    spdlog::debug("MyExo::configureMasterPDOs()");
    m_robot->configureMasterPDOs();
}


void MyExo::update() {
    
    time = (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - time0)
                .count()) /
           1e6;
    spdlog::trace("Update()");
    StateMachine::update();
    m_logger.recordLogData();
}