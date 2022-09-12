#include "Robot.h"

Robot::Robot(std::size_t size)
{
    jointPositions_ = Eigen::VectorXd::Zero(size);
    jointVelocities_ = Eigen::VectorXd::Zero(size);
    jointTorques_ = Eigen::VectorXd::Zero(size);
}

Robot::~Robot() {
    spdlog::debug("Robot object deleted");
}

bool Robot::initialise() {
    return initialiseNetwork();
}

bool Robot::disable() {
    for (auto p : joints) {
        p->disable();
    }
    spdlog::info("X2Robot: Disabled robot");
    return true;
}

void Robot::updateRobot() {

    //Retrieve latest values from hardware
    for (auto joint : joints)
        joint->updateValue();
    for (auto input : inputs)
        input->updateInput();

    //Update local copies of joint values
    for (std::size_t i = 0; i < joints.size(); i++)
    {
        jointPositions_[i] = joints[i]->getPosition();
        jointVelocities_[i] = joints[i]->getVelocity();
        jointTorques_[i] = joints[i]->getTorque();
    }
}

const Eigen::VectorXd& Robot::getPosition() {
    return jointPositions_;
}

const Eigen::VectorXd& Robot::getVelocity() {
    return jointVelocities_;
}

const Eigen::VectorXd& Robot::getTorque() {
    return jointTorques_;
}

bool Robot::configureMasterPDOs() {
    for (auto j : joints) {
        j->configureMasterPDOs();
    }
    for (auto i : inputs) {
        i->configureMasterPDOs();
    }
    return true;
}