#ifndef EXO_STATE_HPP
#define EXO_STATE_HPP

/* CORC interface */
#include "State.h"
#include "X2Robot.h"

// Set state
class SetState : public State
{
public: // Constructors / Destructors
    explicit SetState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot);
    SetState(const SetState&) = delete;
    SetState(SetState &&) = delete;
    ~SetState();

public: // Override methods
    void entry() override;
    void during() override;
    void exit() override;

private: // Robot access
    std::shared_ptr<X2Robot> _Robot;
};

// Run state
class RunState : public State
{
public: // Constructors / Destructors
    explicit RunState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot);
    RunState(const RunState &) = delete;
    RunState(RunState &&) = delete;
    ~RunState();

public: // Override methods
    void entry() override;
    void during() override;
    void exit() override;

private: // Robot access
    std::shared_ptr<X2Robot> _Robot;
};

// Off state
class OffState : public State
{
public: // Constructors / Destructors
    explicit OffState(StateMachine *state_machine, std::shared_ptr<X2Robot> robot);
    OffState(const OffState &) = delete;
    OffState(OffState &&) = delete;
    ~OffState();

public: // Override methods
    void entry() override;
    void during() override;
    void exit() override;

private: // Robot access
    std::shared_ptr<X2Robot> _Robot;
};

#endif