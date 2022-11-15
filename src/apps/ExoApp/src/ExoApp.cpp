#include "ExoApp.hpp"
#define LOG(x)  spdlog::info("[ExoApp]: {}", x)
#define APP(x)  static_cast<ExoApp &>(x)

/*****************************
 * STATE MACHINE TRANSITIONS *
 *****************************/
ExoApp::ExoApp(int argc, char **argv) : StateMachine()
{
    LOG("Spawning X2  Robot...(1/3)");
    setRobot(std::make_shared<X2Robot>("exo"));

    LOG("Spawning Exo Node ...(2/3)");
    rclcpp::InitOptions ros_init = rclcpp::InitOptions();
    ros_init.shutdown_on_sigint = false;
    rclcpp::init(argc, argv, ros_init);
    _Node = std::make_shared<ExoNode>(get_robot());

    LOG("Spawning Run State...(3/3)");
    addState("RunState", std::make_unique<RunState>(get_robot(), _Node));

    setInitState("RunState");

    LOG("Ready");
}

void ExoApp::init()
{
    get_robot()->initialiseNetwork();
}

void ExoApp::end()
{
    LOG("Shut down");
}

void ExoApp::hwStateUpdate()
{
#ifdef DEBUG
    spdlog::info("positions: [{:.4}, {:.4}, {:.4}, {:.4}]",
        get_robot()->getPosition()[0],
        get_robot()->getPosition()[1],
        get_robot()->getPosition()[2],
        get_robot()->getPosition()[3]
    );
#endif
    get_robot()->updateRobot();
    rclcpp::spin_some(get_node()->get_interface());
}

bool ExoApp::configureMasterPDOs()
{
    return get_robot()->configureMasterPDOs();
}

std::shared_ptr<X2Robot> ExoApp::get_robot()
{
    return std::static_pointer_cast<X2Robot>(_robot);
}

std::shared_ptr<ExoNode> ExoApp::get_node()
{
    return _Node;
}