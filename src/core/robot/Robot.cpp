#include "Robot.h"

short int sign(double val) { return (val > 0) ? 1 : ((val < 0) ? -1 : 0); }

Robot::~Robot() {
    spdlog::debug("Robot object deleted");
}

void Robot::init(std::size_t size)
{
    jointPositions_ = Eigen::VectorXd::Zero(size);
    jointVelocities_ = Eigen::VectorXd::Zero(size);
    jointTorques_ = Eigen::VectorXd::Zero(size);
}

bool Robot::initialiseFromYAML(std::string yaml_config_file) {
    if(yaml_config_file.size()>0) {
        // need to use address of base directory because when run with ROS, working directory is ~/.ros
        //std::string baseDirectory = XSTR(BASE_DIRECTORY);
        //std::string relativeFilePath = "/config/";
        try {
            YAML::Node params = YAML::LoadFile(yaml_config_file);

            if(!params[robotName]){
                spdlog::error("Parameters of {} couldn't be found in {} !", robotName, yaml_config_file);
                spdlog::error("Default parameters used !");

                return false;
            }
            else {
                //Attempt to load parameters from YAML file (delegated to each custom robot implementation)
                return loadParametersFromYAML(params);
            }

        } catch (...) {
            spdlog::error("Failed loading parameters from {}. Using default parameters instead.", yaml_config_file);
            return false;
        }
    }
    else {
        spdlog::info("Using default robot parameters (no YAML file specified).");
        return false;
    }
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

void Robot::printStatus() {
    std::cout << "q=[ " << jointPositions_.transpose() * 180 / M_PI << " ]\t";
    //std::cout << "dq=[ " << jointVelocities_.transpose() * 180 / M_PI << " ]\t";
    //std::cout << "tau=[ " << jointTorques_.transpose() << " ]\t";
    std::cout << std::endl;
}

void Robot::printJointStatus(int J_i) {
    joints[J_i]->printStatus();
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
