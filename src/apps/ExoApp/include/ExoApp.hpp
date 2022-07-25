#ifndef EXO_APP_HPP
#define EXO_APP_HPP

#include "X2Robot.h"
#include "StateMachine.h"

#include "ExoState.hpp"
#include "rclcpp/rclcpp.hpp"

class ExoApp : public StateMachine
{
public:
    ExoApp(int argc, char **argv, const float period);
    ExoApp(const ExoApp &) = delete;
    ExoApp(ExoApp &&) = delete;
    ~ExoApp();

public:
    void init();
    void end();
    void update();
    void hwStateUpdate();
    bool configureMasterPDOs();

public:
    bool running;

private:
    EventObject(ToRun) *_ToRun;
    EventObject(ToOff) *_ToOff;
    RunState *_RunState;
    OffState *_OffState;
    std::shared_ptr<X2Robot> _Robot;
};

#endif