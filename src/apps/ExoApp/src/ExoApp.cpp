#include "ExoApp.hpp"
#define LOG(x)  spdlog::info("[ExoApp]: {}", x)
#define APP(x)  static_cast<ExoApp &>(x)

/************************
 * STATE MACHINE EVENTS *
 ************************/
bool off_to_set(StateMachine &sm)
{
    return APP(sm).get_node()->get_heart_beat().status == ExoNode::TMP;
}

bool run_to_off(StateMachine &sm)
{
    return !APP(sm).get_robot()->safetyCheck() || !APP(sm).get_node()->ok();
}

bool run_to_set(StateMachine &sm)
{
    return APP(sm).get_node()->overwrite_save();
}

bool set_to_off(StateMachine &sm)
{
    return APP(sm).get_node()->save_error() || !APP(sm).get_node()->ok();
}

bool set_to_run(StateMachine &sm)
{
    return APP(sm).get_node()->is_saved();
}

/*****************************
 * STATE MACHINE TRANSITIONS *
 *****************************/
ExoApp::ExoApp(int argc, char **argv) : StateMachine()
{
    LOG("Spawning X2  Robot...(1/5)");
    setRobot(std::make_shared<X2Robot>("exo"));

    LOG("Spawning Exo Node ...(2/5)");
    rclcpp::InitOptions ros_init = rclcpp::InitOptions();
    ros_init.shutdown_on_sigint = false;
    rclcpp::init(argc, argv, ros_init);
    _Node = std::make_shared<ExoNode>(get_robot());

    LOG("Spawning Off State...(3/5)");
    addState("OffState", std::make_unique<OffState>(get_robot(), _Node));

    LOG("Spawning Run State...(4/5)");
    addState("RunState", std::make_unique<RunState>(get_robot(), _Node));

    LOG("Spawning Set State...(5/5)");
    addState("SetState", std::make_unique<SetState>(get_robot(), _Node));

    addTransition("OffState", off_to_set, "SetState");
    addTransition("RunState", run_to_off, "OffState");
    addTransition("RunState", run_to_set, "SetState");
    addTransition("SetState", set_to_off, "OffState");
    addTransition("SetState", set_to_run, "RunState");
    setInitState("SetState");

    LOG("Ready");

    // FIXME:
    get_node()->set_is_saved(false);
}

void ExoApp::init()
{
    get_robot()->initialiseNetwork();

    
}

void ExoApp::end()
{
    // TODO:
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
    get_node()->publish_heart_beat();
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