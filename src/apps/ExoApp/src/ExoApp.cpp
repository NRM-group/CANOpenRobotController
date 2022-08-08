#include "ExoApp.hpp"

ExoApp::ExoApp(int argc, char **argv, const float period) : running(false)
{
    // ROS initialisation
    rclcpp::InitOptions ros_init = rclcpp::InitOptions();
    ros_init.shutdown_on_sigint = false;
    rclcpp::init(argc, argv, ros_init);
    spdlog::info("ExoApp: Initialised ROS context (1/3)");
    // State machine setup
    _Robot = std::make_shared<X2Robot>();
    _ToRun = new ToRun(this);
    _ToOff = new ToOff(this);
    _RunState = new RunState(this, _Robot);
    _OffState = new OffState(this, _Robot);
    NewTransition(_OffState, _ToRun, _RunState);
    NewTransition(_RunState, _ToOff, _OffState);
    spdlog::info("ExoApp: Initialised CORC states (2/3)");
    // ROS parameters
    std::string x2_params;
    _RunState->declare_parameter("x2_params");
    _RunState->get_parameter("x2_params", x2_params);
    spdlog::info("ExoApp: Initialised ROS parameters (3/3)");
    // State machine start
    _Robot->init(period, _RunState->get_name(), x2_params);
    spdlog::info("ExoApp: Ready");
}

ExoApp::~ExoApp()
{
    delete _RunState;
    delete _OffState;
    rclcpp::shutdown();
}

void ExoApp::init()
{
    StateMachine::initialize(_RunState);
    running = _Robot->initialise();
    if (running)
    {
        hwStateUpdate();
        spdlog::info("ExoApp: Call to init() successful");
    }
    else
    {
        spdlog::error("ExoApp: Call to init() failed");
    }
}

void ExoApp::end()
{
    if (running)
    {
        currentState->exit();
        _Robot->disable();
    }
    spdlog::info("ExoApp: Call to end()");
}

void ExoApp::update()
{
    StateMachine::update();
}

void ExoApp::hwStateUpdate()
{
    _Robot->updateRobot();
}

bool ExoApp::configureMasterPDOs()
{
    if (_Robot->configureMasterPDOs())
    {
        spdlog::info("Configured master PDOs");
        return true;
    }
    return false;
}

bool ExoApp::ToRun::check()
{
    return false;
}

bool ExoApp::ToOff::check()
{
    return false;
}