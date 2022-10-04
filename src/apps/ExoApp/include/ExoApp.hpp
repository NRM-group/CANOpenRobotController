#ifndef __EXO_APP_HPP__
#define __EXO_APP_HPP__

/* Core */
#include "StateMachine.h"
/* Hardware */
#include "X2Robot.h"
/* Apps */
#include "ExoState.hpp"
#include "ExoNode.hpp"
/* ROS2 */
#include <rclcpp/rclcpp.hpp>

class ExoApp : public StateMachine
{
public:
    ExoApp(int argc, char **argv);

public:
    void init() override;
    void end() override;
    void hwStateUpdate() override;
    bool configureMasterPDOs() override;

public:
    std::shared_ptr<X2Robot> get_robot();
    std::shared_ptr<ExoNode> get_node();

private:
    std::shared_ptr<ExoNode> _Node;
};

#endif//__EXO_APP_HPP__