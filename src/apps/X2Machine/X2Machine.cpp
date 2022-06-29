#include "X2Machine.h"

#define OWNER ((X2Machine *)owner)

X2Machine::X2Machine(int argc, char** argv, const float updateT) {

    // specify custom SIGINT handler is going to be used
    rclcpp::InitOptions rosInit = rclcpp::InitOptions();
    rosInit.shutdown_on_sigint = false;

    // initalise ROS2 node
    rclcpp::init(argc, argv, rosInit);
    node_ = rclcpp::Node::make_shared("x2");

    // get robot name from node name and remove '/'
    robotName_ = "x2";

#ifdef SIM
    robot_ = new X2Robot(node updateT, robotName_);
#else
    std::string param_file;
    node_->declare_parameter("x2_params");
    node_->get_parameter("x2_params", param_file);
    robot_ = new X2Robot(updateT, robotName_, param_file);
#endif

    // Create PRE-DESIGNED State Machine events and state objects
    startExo = new StartExo(this);
    exitSafe = new ExitSafe(this);

    // create state objects
    x2FollowerState_ = new X2FollowerState(this, robot_, updateT);
    x2SafetyState_ = new X2SafetyState(this, robot_, updateT);
    
    x2MachineRos2_ = new X2MachineROS2(robot_, x2FollowerState_, node_);

    //Transition Events
    NewTransition(x2FollowerState_, exitSafe, x2SafetyState_);
    NewTransition(x2SafetyState_, startExo, x2FollowerState_);
}

void X2Machine::init() {
    spdlog::info("X2 Initalised");

    // create states with ROS features
    x2FollowerState_->csvFileName = getGaitCycle();
    StateMachine::initialize(x2FollowerState_);
    initialised = robot_->initialise();
    time0 = std::chrono::steady_clock::now();

    running = true;
}

void X2Machine::end() {
    if (initialised) {
        currentState->exit();
        robot_->disable();
        delete x2MachineRos2_;
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

bool X2Machine::ExitSafe::check(void) {
    //TODO: Need a seperate Safety management program
    if (OWNER->x2FollowerState_->checkSafety()) {
        spdlog::warn("Excessive Torque detected, transitioning to safe state");
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
    x2MachineRos2_->update();
    rclcpp::spin_some(node_);
}

bool X2Machine::configureMasterPDOs() {
    spdlog::info("X2 PDOs Configured");
    return robot_->configureMasterPDOs();
}

std::string X2Machine::getGaitCycle(void) {
    std::string buffer;
    node_->declare_parameter("walking_gait");
    node_->get_parameter("walking_gait", buffer);

    buffer.erase(remove(buffer.begin(), buffer.end(), ' '), buffer.end());
    buffer.erase(remove(buffer.begin(), buffer.end(), '\n'), buffer.end());

    return buffer;
}