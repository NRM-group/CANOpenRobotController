#include "ExoNode.hpp"
#define LOG(x)  spdlog::info("[ExoNode]: {}", x)

/***************
 * CONSTRUCTOR *
 ***************/
ExoNode::ExoNode(std::shared_ptr<X2Robot> robot)
    : Node(robot->getRobotName()), _Robot(robot), _TorqueLimit()
{
    _PubJointState = create_publisher<JointState>("joint_states", 4);
    _PubJointReference = create_publisher<FloatArray>("joint_references", 4);
    _PubJointVelReference = create_publisher<FloatArray>("joint_vel_references", 4);
    _PubJointAccelReference = create_publisher<FloatArray>("joint_accel_references", 4);
    _PubAffcTorque = create_publisher<FloatArray>("affc_torque", 4);
    _PubAffcLeftUpperKnownParameters = create_publisher<FloatArray>("affc_left_upper_known_parameters", 4);
    _PubAffcLeftLowerKnownParameters = create_publisher<FloatArray>("affc_left_lower_known_parameters", 4);
    _PubAffcRightUpperKnownParameters = create_publisher<FloatArray>("affc_right_upper_known_parameters", 4);
    _PubAffcRightLowerKnownParameters = create_publisher<FloatArray>("affc_right_lower_known_parameters", 4);
    _PubAffcLeftUnknownParameters = create_publisher<FloatArray>("affc_left_unknown_parameters", 4);
    _PubAffcRightUnknownParameters = create_publisher<FloatArray>("affc_right_unknown_parameters", 4);
    _PubAffcTrackingError = create_publisher<FloatArray>("affc_tracking_error", 4);
    _PubAffcSGDError = create_publisher<FloatArray>("affc_sgd_error", 4);

    _SubTorqueLimit = create_subscription<Float>(
        "torque_limit", 4,
        std::bind(&ExoNode::torque_limit_callback, this, _1)
    );

    declare_parameter<int>("dry_run", 0);
    declare_parameter<std::string>("exo_file", "");
    declare_parameter<std::string>("gait_file", "");
    declare_parameter<std::string>("affc_file", "");

    declare_parameter<double>("max_torque", 0.0);
    declare_parameter<double>("max_velocity", 0.0);
    declare_parameter<double>("position_limits.max_hip", 0.0);
    declare_parameter<double>("position_limits.min_hip", 0.0);
    declare_parameter<double>("position_limits.max_knee", 0.0);
    declare_parameter<double>("position_limits.min_knee", 0.0);

    get_parameter<double>(
        "max_torque",
        _Robot->getRobotParameters().maxTorque
    );
    get_parameter<double>(
        "max_velocity",
        _Robot->getRobotParameters().maxVelocity
    );
    get_parameter<double>(
        "position_limits.max_hip",
        _Robot->getRobotParameters().jointPositionLimits.hipMax
    );
    get_parameter<double>(
        "position_limits.min_hip",
        _Robot->getRobotParameters().jointPositionLimits.hipMin
    );
    get_parameter<double>(
        "position_limits.max_knee",
        _Robot->getRobotParameters().jointPositionLimits.kneeMax
    );
    get_parameter<double>(
        "position_limits.min_knee",
        _Robot->getRobotParameters().jointPositionLimits.kneeMin
    );
}

/**************
 * VISIBILITY *
 **************/
void ExoNode::ros_declare(const std::vector<std::string> &names)
{
    for (const std::string &n : names) {
        declare_parameter<std::vector<double>>(n, { });
    }
}

void ExoNode::ros_parameter(const std::string &name, std::vector<double> &val)
{
    get_parameter<std::vector<double>>(name, val);
}

void ExoNode::get_exo_file(std::string &path)
{
    get_parameter<std::string>("exo_file", path);
}

void ExoNode::get_gait_file(std::string &path)
{
    get_parameter<std::string>("gait_file", path);
}

void ExoNode::get_affc_file(std::string &path)
{
    get_parameter<std::string>("affc_file", path);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ExoNode::get_interface()
{
    return get_node_base_interface();
}

/**************
 * PUBLISHERS *
 **************/
void ExoNode::publish_joint_reference(const std::vector<double> &val)
{
    // TODO: May need to be offset + 1
    FloatArray msg{};

    msg.data = std::move(val);

    _PubJointReference->publish(msg);
}

void ExoNode::publish_joint_vel_reference(const std::vector<double> &val) 
{
    FloatArray msg{};

    msg.data = std::move(val);

    _PubJointVelReference->publish(msg);
}

void ExoNode::publish_joint_accel_reference(const std::vector<double> &val)
{
    FloatArray msg{};

    msg.data = std::move(val);

    _PubJointAccelReference->publish(msg);
}

void ExoNode::publish_joint_state()
{
    JointState msg{};

    const std::array<std::string, X2_NUM_JOINTS> name {
        "left_hip", "left_knee", "right_hip", "right_knee"
    };

    msg.header.stamp = now();
    msg.name.assign(name.cbegin(), name.cend());
    msg.position.assign(
        _Robot->getPosition().data(),
        _Robot->getPosition().data() + _Robot->getPosition().size()
    );
    msg.velocity.assign(
        _Robot->getVelocity().data(),
        _Robot->getVelocity().data() + _Robot->getVelocity().size()
    );
    msg.effort.assign(
        _Robot->getTorque().data(),
        _Robot->getTorque().data() + _Robot->getTorque().size()
    );

    _PubJointState->publish(msg);
}

void ExoNode::publish_affc_torque(const std::vector<double> &val)
{
    FloatArray msg{};

    msg.data = std::move(val);

    _PubAffcTorque->publish(msg);
}

void ExoNode::publish_affc_known_parameters(const std::vector<double> &leftUpper, 
                                            const std::vector<double> &rightUpper,
                                            const std::vector<double> &leftLower,
                                            const std::vector<double> &rightLower)
{
    FloatArray leftUpperMsg{};
    FloatArray leftLowerMsg{};
    FloatArray rightUpperMsg{};
    FloatArray rightLowerMsg{};

    leftUpperMsg.data = std::move(leftUpper);
    leftLowerMsg.data = std::move(leftLower);
    rightUpperMsg.data = std::move(rightUpper);
    rightLowerMsg.data = std::move(rightLower);

    _PubAffcLeftUpperKnownParameters->publish(leftUpperMsg);
    _PubAffcLeftLowerKnownParameters->publish(leftLowerMsg);
    _PubAffcRightUpperKnownParameters->publish(rightUpperMsg);
    _PubAffcRightLowerKnownParameters->publish(rightLowerMsg);
}

void ExoNode::publish_affc_unknown_parameters(const std::vector<double> &left,
                                              const std::vector<double> &right)
{
    FloatArray leftMsg{};
    FloatArray rightMsg{};

    leftMsg.data = std::move(left);
    rightMsg.data = std::move(right);

    _PubAffcLeftUnknownParameters->publish(leftMsg);
    _PubAffcRightUnknownParameters->publish(rightMsg);
}

void ExoNode::publish_affc_tracking_error(const std::vector<double> &val)
{
    FloatArray msg{};

    msg.data = std::move(val);

    _PubAffcTrackingError->publish(msg); 
}

void ExoNode::publish_affc_sgd_error(const std::vector<double> &val)
{
    FloatArray msg{};

    msg.data = std::move(val);

    _PubAffcSGDError->publish(msg);
}

const double ExoNode::get_torque_limit() const
{
    return _TorqueLimit;
}

/*************
 * CALLBACKS *
 *************/
void ExoNode::torque_limit_callback(const Float::SharedPtr msg)
{
    _TorqueLimit = msg->data;
}