#include "X2SafetyState.hpp"
/**
 * @file X2SafetyState.cpp
 * @author your name (you@domain.com)
 * @brief If the torque exceeds limit, the system will transition here which will
 *  lock the joints in position
 * @version 0.1
 * @date 2022-06-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */


X2SafetyState::X2SafetyState(StateMachine* m, X2Robot* exo, const float updateT, const char* name) : 
    State(m,name), robot_(exo), freq_(1 / updateT) {
    //Constructor for safety state
    stationaryVelocity = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
}

void X2SafetyState::entry(void) {
    spdlog::info("Entered Safety State");
}

void X2SafetyState::during(void) {
    //Lock the joints in their current position, (zero velocity)
    if (robot_->getControlMode() != CM_VELOCITY_CONTROL) {
        robot_->initVelocityControl();
    }
    robot_->setVelocity(stationaryVelocity);

}

void X2SafetyState::exit(void) {
    //Safe to exit transitions in zero torque mode.
    spdlog::info("Exiting Safety State");
    robot_->initTorqueControl();
    robot_->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
}