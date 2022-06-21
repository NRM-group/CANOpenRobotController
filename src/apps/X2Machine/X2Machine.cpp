#include "X2Machine.h"

#define OWNER ((X2Machine *)owner)

X2Machine::X2Machine(int argc, char* argv[], const float updateT) {

    ros::init(argc, argv, "x2", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");

    // get robot name from node name and remove '/'
    robotName_ = ros::this_node::getName();
    robotName_.erase(0, 1);

#ifdef SIM
    robot_ = new X2Robot(nodeHandle, updateT, robotName_);
#else
    robot_ = new X2Robot(updateT, robotName_);
#endif

    // Create pre-designed state machine events and state objects
    startExo = new StartExo(this);

    // create state object
    x2FollowerState_ = new X2FollowerState(this, robot_, updateT);

    // create ros object
    x2MachineRos_ = new X2MachineROS(robot_, x2FollowerState_, nodeHandle);
}

void X2Machine::init() {
    spdlog::info("X2 Initalised");

    // create states with ROS features
    StateMachine::initialize(x2FollowerState_);

    initialised = robot_->initialise();
    time0 = std::chrono::steady_clock::now();

    running = true;
}

void X2Machine::end() {
    if (initialised) {
        currentState->exit();
        robot_->disable();
        delete x2MachineRos_;
        delete robot_;
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool X2Machine::StartExo::check(void) {
    if (OWNER->robot_->keyboard->getS() == true) {
        spdlog::info("X2 Started");
        return true;
    }

    return false;
}

void X2Machine::hwStateUpdate(void) {
    robot_->updateRobot();
}

void X2Machine::update() {
    time = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time0).count()) / 1e6;

    StateMachine::update();
    x2MachineRos_->update();
    ros::spinOnce();
}

bool X2Machine::configureMasterPDOs() {
    spdlog::info("X2 PDOs Configured");
    return robot_->configureMasterPDOs();
}