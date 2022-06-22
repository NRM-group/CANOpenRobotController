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
    robotName_ = node_->get_name();
    robotName_.erase(0, 1);

#ifdef SIM
    robot_ = new X2Robot(node updateT, robotName_);
#else
    robot_ = new X2Robot(updateT, robotName_);
#endif

    // Create PRE-DESIGNED State Machine events and state objects
    startExo = new StartExo(this);

    // create state object
    x2FollowerState_ = new X2FollowerState(this, robot_, updateT);

    x2MachineRos2_ = new X2MachineROS2(robot_, x2FollowerState_, node_);
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
    rclcpp::Parameter str_param;
    std::string str;
    node_->declare_parameter("walking_gait");
    node_->get_parameter("walking_gait", str);


    str.erase(remove(str.begin(), str.end(), ' '), str.end());
    str.erase(remove(str.begin(), str.end(), '\n'), str.end());


    return str;   
}