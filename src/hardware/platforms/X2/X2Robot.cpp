#include "X2Robot.h"

/**
 * An enum type.
 * Joint Index for the 4 joints (note, CANopen NODEID = this + 1)
 */
enum X2Joints {
    X2_LEFT_HIP = 0,   /**< Left Hip*/
    X2_LEFT_KNEE = 1,  /**< Left Knee*/
    X2_RIGHT_HIP = 2,  /**< Right Hip*/
    X2_RIGHT_KNEE = 3, /**< Right Knee*/
};

/**
 * Paramater definitions: Hip motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs hipJDP{
        250880,       // drivePosA
        0,            // drivePosB
        deg2rad(90),  //jointPosA
        deg2rad(0)    //jointPosB
};
/**
 * Paramater definitions: Knee motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs kneeJDP{
        0,       // drivePosA
        250880,            //drivePosB
        deg2rad(0),  //jointPosA
        deg2rad(-90)    //jointPosB
};

static volatile sig_atomic_t exitLoop = 0;

#ifdef SIM
X2Robot::X2Robot(ros::NodeHandle &nodeHandle, std::string robot_name, std::string yaml_config_file):Robot(robot_name, yaml_config_file), x2Parameters()
#else
X2Robot::X2Robot(std::string robot_name, std::string yaml_config_file):Robot(robot_name, yaml_config_file), x2Parameters()
#endif
    {

    spdlog::debug("{} Created", robotName);

#ifdef NOROBOT
    simJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    simJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    simJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    simJointTorquesViaStrainGauges_ = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);
    simInteractionForces_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    simGroundReactionForces_ = Eigen::VectorXd::Zero(X2_NUM_GRF_SENSORS);
    simBackPackAngleOnMedianPlane_ = 0.0;
    simBackPackAngularVelocityOnMedianPlane_ = 0.0;
    simContactAnglesOnMedianPlane_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
#endif

    controlMode = ControlMode::CM_UNCONFIGURED;

    // Initializing the parameters to zero
    x2Parameters.m = Eigen::VectorXd::Zero(X2_NUM_JOINTS+2);
    x2Parameters.l = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.imuDistance = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.s = Eigen::VectorXd::Zero(X2_NUM_JOINTS+3);
    x2Parameters.I = Eigen::VectorXd::Zero(X2_NUM_JOINTS+2);
    x2Parameters.G = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.c0 = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.c1 = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.c2 = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.cuffWeights = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);
    x2Parameters.forceSensorScaleFactor = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);
    x2Parameters.grfSensorScaleFactor = Eigen::VectorXd::Zero(X2_NUM_GRF_SENSORS);
    x2Parameters.grfSensorThreshold = Eigen::VectorXd::Zero(X2_NUM_GRF_SENSORS);
    x2Parameters.imuParameters.useIMU = false;

	// IMPORTANT DEFAULT PARAMETERS
    x2Parameters.maxVelocity = 3.0;
    x2Parameters.maxTorque = 70.0;
	x2Parameters.jointPositionLimits.hipMax = 120;
	x2Parameters.jointPositionLimits.hipMin = -40;
	x2Parameters.jointPositionLimits.kneeMax = 0;
	x2Parameters.jointPositionLimits.kneeMin = -120;

    jointTorquesViaStrainGauges_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

    interactionForces_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    smoothedInteractionForces_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    previousSmoothedInteractionForces_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);

    groundReactionForces_ = Eigen::VectorXd::Zero(X2_NUM_GRF_SENSORS);
    backpackQuaternions_ = Eigen::VectorXd::Zero(4);
    backpackGyroData_ = Eigen::VectorXd::Zero(3);
    backPackAngleOnMedianPlane_ = 0.0;
    backPackAngularVelocityOnMedianPlane_ = 0.0;
    contactAnglesOnMedianPlane_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

    gaitState_ = GaitState::FLYING;

    generalizedAccByDerivative_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    filteredGeneralizedAccByDerivative_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    previousFilteredGeneralizedAccByDerivative_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    previousJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

    feedForwardTorque_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    massMatrix_ = Eigen::MatrixXd::Zero(X2_NUM_GENERALIZED_COORDINATES,X2_NUM_GENERALIZED_COORDINATES);

    gravitationTorque_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);

    corriolisTorque_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    frictionTorque_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    estimatedGeneralizedAcceleration_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    selectionMatrix_ = Eigen::MatrixXd::Zero(X2_NUM_JOINTS, X2_NUM_GENERALIZED_COORDINATES);
    selectionMatrix_ << Eigen::MatrixXd::Zero(X2_NUM_JOINTS, 1) , Eigen::MatrixXd::Identity(X2_NUM_JOINTS,X2_NUM_JOINTS);
    pseudoInverseOfSelectionMatrixTranspose_ = selectionMatrix_.transpose().completeOrthogonalDecomposition().pseudoInverse(); // calculated beforehand because it is a computationally expensive operation

    jointVelDerivativeCutOffFreq_ = 0.0; // updated from from slider
    backpackVelDerivativeCutOffFreq_ = 0.0;

    //Check if YAML file exists and contain robot parameters
    //initialiseFromYAML(yaml_config_file);

    // spdlog::debug("initialiseJoints call");
    initialiseJoints();
    initialiseInputs();
#ifdef SIM
    initialiseROS(nodeHandle);
#endif
}

X2Robot::~X2Robot() {
//    if(x2Parameters.imuParameters.useIMU) technaidIMUs->exit();
    freeMemory();
    spdlog::debug("X2Robot deleted");
}

void X2Robot::signalHandler(int signum) {
    exitLoop = 1;
    std::raise(SIGTERM); //Clean exit
}

#ifdef SIM
void X2Robot::initialiseROS(ros::NodeHandle &nodeHandle) {
    controllerSwitchClient_ = nodeHandle.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

    positionCommandPublisher_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("position_controller/command", 10);
    velocityCommandPublisher_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("velocity_controller/command", 10);
    torqueCommandPublisher_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("torque_controller/command", 10);

    jointStateSubscriber_ = nodeHandle.subscribe("joint_states", 1, &X2Robot::jointStateCallback, this);
}
#endif

void X2Robot::resetErrors() {
    spdlog::debug("Clearing errors on all motor drives ");
    for (auto p : joints) {
        // Put into ReadyToSwitchOn()
        p->resetErrors();
    }
}

bool X2Robot::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }

#ifdef SIM
    controllerSwitchMsg_.request.start_controllers = {"position_controller"};
    controllerSwitchMsg_.request.stop_controllers = {"velocity_controller", "torque_controller"};
    controllerSwitchMsg_.request.strictness = 1;
    controllerSwitchMsg_.request.start_asap = true;
    controllerSwitchMsg_.request.timeout = 0.0;

    if (controllerSwitchClient_.call(controllerSwitchMsg_)) {
        spdlog::info("Switched to position controller");
    } else {
        spdlog::error("Failed switching to position controller");
        returnValue = false;
    }
#endif

    if(returnValue) controlMode = ControlMode::CM_POSITION_CONTROL;

    return returnValue;
}

bool X2Robot::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_VELOCITY_CONTROL, velControlMotorProfile) != CM_VELOCITY_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(10000);
    for (auto p : joints) {
        p->enable();
    }

#ifdef SIM
    controllerSwitchMsg_.request.start_controllers = {"velocity_controller"};
    controllerSwitchMsg_.request.stop_controllers = {"position_controller", "torque_controller"};
    controllerSwitchMsg_.request.strictness = 1;
    controllerSwitchMsg_.request.start_asap = true;
    controllerSwitchMsg_.request.timeout = 0.0;

    if (controllerSwitchClient_.call(controllerSwitchMsg_)) {
        spdlog::info("Switched to velocity controller");
    } else {
        spdlog::error("Failed switching to velocity controller");
        returnValue = false;
    }
#endif

    if(returnValue) controlMode = ControlMode::CM_VELOCITY_CONTROL;

    return returnValue;
}

bool X2Robot::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }

#ifdef SIM
    controllerSwitchMsg_.request.start_controllers = {"torque_controller"};
    controllerSwitchMsg_.request.stop_controllers = {"position_controller", "velocity_controller"};
    controllerSwitchMsg_.request.strictness = 1;
    controllerSwitchMsg_.request.start_asap = true;
    controllerSwitchMsg_.request.timeout = 0.0;

    if (controllerSwitchClient_.call(controllerSwitchMsg_)) {
        spdlog::info("Switched to torque controller");
    } else {
        spdlog::error("Failed switching to torque controller");
        returnValue = false;
    }
#endif

    if(returnValue) controlMode = ControlMode::CM_TORQUE_CONTROL;

    return returnValue;
}

setMovementReturnCode_t X2Robot::setPosition(Eigen::VectorXd positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        spdlog::debug("Joint {}, Target {}, Current {}", i, positions[i], ((X2Joint *)p)->getPosition());
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setPosition(positions[i]);
        if (setPosCode == INCORRECT_MODE) {
            spdlog::error("Joint {} is not in Position Control ", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} Unknown Error", p->getId());
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }

#ifdef SIM
    std::vector<double> positionVector(X2_NUM_JOINTS);

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        positionVector[i] = positions[i];
    }

    positionCommandMsg_.data = positionVector;
    positionCommandPublisher_.publish(positionCommandMsg_);
#elif NOROBOT
    simJointPositions_ = positions;
#endif

    return returnValue;
}

setMovementReturnCode_t X2Robot::setVelocity(Eigen::VectorXd velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setVelocity(velocities[i]);
        if (setPosCode == INCORRECT_MODE) {
            spdlog::error("Joint {} is not in Velocity Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} Unknown Error", p->getId());
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }

#ifdef SIM
    std::vector<double> velocityVector(X2_NUM_JOINTS);

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        velocityVector[i] = velocities[i];
    }

    velocityCommandMsg_.data = velocityVector;
    velocityCommandPublisher_.publish(velocityCommandMsg_);
#endif

    return returnValue;
}

setMovementReturnCode_t X2Robot::setTorque(Eigen::VectorXd torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setTorque(torques[i]);
        if (setPosCode == INCORRECT_MODE) {
            spdlog::error("Joint {} is not in Torque Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} Unknown Error", p->getId());
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }

#ifdef SIM
    std::vector<double> torqueVector(X2_NUM_JOINTS);

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        torqueVector[i] = torques[i];
    }

    torqueCommandMsg_.data = torqueVector;
    torqueCommandPublisher_.publish(torqueCommandMsg_);
#endif

    return returnValue;
}

// bool X2Robot::calibrateForceSensors() {
//     int numberOfSuccess = 0;
//     for (int i = 0; i < X2_NUM_FORCE_SENSORS + X2_NUM_GRF_SENSORS; i++) {
//         if (forceSensors[i]->calibrate()) numberOfSuccess++;
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(3000));

//     if (numberOfSuccess == X2_NUM_FORCE_SENSORS + X2_NUM_GRF_SENSORS) {
//         spdlog::info("[X2Robot::calibrateForceSensors]: Zeroing of force sensors are successfully completed.");
//         return true;
//     } else {
//         spdlog::error("[X2Robot::calibrateForceSensors]: Zeroing failed.");
//         return false;
//     }
// }

bool X2Robot::calibrateForceSensors() {

    for (std::size_t i = 0; i < X2_NUM_FORCE_SENSORS; i++) {
        forceSensors[i]->calibrate();
    }
    return true;
}

Eigen::VectorXd X2Robot::getBackpackQuaternions() {

    for(int imuIndex = 0; imuIndex<numberOfIMUs_; imuIndex++){
        if(x2Parameters.imuParameters.location[imuIndex] == 'b'){
            backpackQuaternions_ = technaidIMUs->getQuaternion().col(imuIndex);
        }
    }
    return backpackQuaternions_;
}

Eigen::VectorXd X2Robot::getBackpackGyroData() {

    for(int imuIndex = 0; imuIndex<numberOfIMUs_; imuIndex++){
        if(x2Parameters.imuParameters.location[imuIndex] == 'b'){
            backpackGyroData_ = technaidIMUs->getAngularVelocity().col(imuIndex);
        }
    }
    return backpackGyroData_;
}

void X2Robot::updateBackpackAndContactAnglesOnMedianPlane() {

    if(!x2Parameters.imuParameters.useIMU){ // if IMU not used set backpack angle to zero
        backPackAngleOnMedianPlane_ = M_PI_2;
        return;
    }

    Eigen::Quaterniond q;
    Eigen::MatrixXd quatEigen = getBackpackQuaternions();
    q.x() = quatEigen(0,0);
    q.y() = quatEigen(1,0);
    q.z() = quatEigen(2,0);
    q.w() = quatEigen(3,0);

    Eigen::Matrix3d R_AD = q.toRotationMatrix();
    double thetaBase = std::asin(R_AD(2,2));

    double BASE_OFFSET = deg2rad(0.0); //TODO: GET THIS offset from COMPARISON OF BACKPACK ANGLE AND CONTACT ANGLE FROM IMUS
    backPackAngleOnMedianPlane_ = -thetaBase - BASE_OFFSET + M_PI/2.0;

    contactAnglesOnMedianPlane_[0] = backPackAngleOnMedianPlane_ + jointPositions_[0];
    contactAnglesOnMedianPlane_[1] = backPackAngleOnMedianPlane_ + jointPositions_[0] + jointPositions_[1];
    contactAnglesOnMedianPlane_[2] = backPackAngleOnMedianPlane_ + jointPositions_[2];
    contactAnglesOnMedianPlane_[3] = backPackAngleOnMedianPlane_ + jointPositions_[2] + jointPositions_[3];
}

void X2Robot::updateBackpackAngularVelocity() {

    if(!x2Parameters.imuParameters.useIMU){ // if IMU not used set backpack angle to zero
        backPackAngularVelocityOnMedianPlane_ = 0.0;
        return;
    }

    // see notebook and TrialMatlabScripts/backpackAngleCalculation.m
    Eigen::Quaterniond q;
    Eigen::MatrixXd quatEigen = getBackpackQuaternions();
    q.x() = quatEigen(0,0);
    q.y() = quatEigen(1,0);
    q.z() = quatEigen(2,0);
    q.w() = quatEigen(3,0);

    Eigen::Matrix3d R_AD = q.toRotationMatrix();

    double alphaBase = std::atan2(-R_AD(0,2), R_AD(1,2));

    Eigen::MatrixXd R_AB(3,3);
    R_AB << 0, cos(alphaBase), -sin(alphaBase),
            0, sin(alphaBase), cos(alphaBase),
            -1, 0, 0;

    Eigen::Vector3d backpackAngularVelocity = R_AB.transpose()*R_AD*getBackpackGyroData();

    backPackAngularVelocityOnMedianPlane_ = -backpackAngularVelocity.y();
}

void X2Robot::updateForceMeasurements() {

    for (int i = 0; i < X2_NUM_FORCE_SENSORS; i++) {
        jointTorquesViaStrainGauges_[i] = forceSensors[i]->getForce();
    }
}

void X2Robot::updateInteractionForce() {

    // this is a very simple approach only valid during flying (no grf). We are currently working on improving this
    interactionForces_ =
            -selectionMatrix_.transpose() * jointTorquesViaStrainGauges_ + gravitationTorque_;

    double alphaDynamicParameters = (2 * M_PI * dt_ * dynamicParametersCutOffFreq_) / (2 * M_PI * dt_ * dynamicParametersCutOffFreq_ + 1); //TODO: get dt from main

    smoothedInteractionForces_ = alphaDynamicParameters*interactionForces_ + (1.0-alphaDynamicParameters)*previousSmoothedInteractionForces_;
    previousSmoothedInteractionForces_ = smoothedInteractionForces_;
}

void X2Robot::updateGeneralizedAcceleration() {

    double alphaJoint = (2*M_PI*dt_*jointVelDerivativeCutOffFreq_)/(2*M_PI*dt_*jointVelDerivativeCutOffFreq_ + 1);
    double alphaBackpack = (2*M_PI*dt_*backpackVelDerivativeCutOffFreq_)/(2*M_PI*dt_*backpackVelDerivativeCutOffFreq_ + 1);

    generalizedAccByDerivative_[0] = (backPackAngularVelocityOnMedianPlane_ - previousBackPackAngularVelocityOnMedianPlane_)/dt_;
    generalizedAccByDerivative_.tail(X2_NUM_JOINTS) = (jointVelocities_ - previousJointVelocities_)/dt_;

    filteredGeneralizedAccByDerivative_[0] = alphaBackpack*generalizedAccByDerivative_[0] +
                                             (1.0 - alphaBackpack)*previousFilteredGeneralizedAccByDerivative_[0];

    filteredGeneralizedAccByDerivative_.tail(X2_NUM_JOINTS) = alphaJoint*generalizedAccByDerivative_.tail(X2_NUM_JOINTS) +
                                                              (1.0 - alphaJoint)*previousFilteredGeneralizedAccByDerivative_.tail(X2_NUM_JOINTS);

    previousFilteredGeneralizedAccByDerivative_ = filteredGeneralizedAccByDerivative_;
    previousJointVelocities_ = jointVelocities_;
    previousBackPackAngularVelocityOnMedianPlane_ = backPackAngularVelocityOnMedianPlane_;

    estimatedGeneralizedAcceleration_ = filteredGeneralizedAccByDerivative_;
}

bool X2Robot::homing(std::vector<int> homingDirection, float thresholdTorque, float delayTime,
                     float homingSpeed, float maxTime) {
    std::vector<bool> success(X2_NUM_JOINTS, false);
    std::chrono::steady_clock::time_point time0;
    signal(SIGINT, signalHandler); // check if ctrl + c is pressed

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        if (homingDirection[i] == 0) continue;  // skip the joint if it is not asked to do homing

        this->initVelocityControl();
        Eigen::VectorXd desiredVelocity;
        desiredVelocity = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        std::chrono::steady_clock::time_point firstTimeHighTorque;  // time at the first time joint exceed thresholdTorque
        bool highTorqueReached = false;

        desiredVelocity[i] = homingSpeed * homingDirection[i] / std::abs(homingDirection[i]);  // setting the desired velocity by using the direction
        time0 = std::chrono::steady_clock::now();

        spdlog::debug("Homing Joint {} ...", i);

        while (success[i] == false &&
               exitLoop == 0 &&
               std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count() < maxTime * 1000) {
            this->updateRobot(true);  // because this function has its own loops, updateRobot needs to be called
            this->setVelocity(desiredVelocity);
            usleep(10000);

            if (std::abs(this->getTorque()[i]) >= thresholdTorque) {  // if high torque is reached
                highTorqueReached = true;
                firstTimeHighTorque = std::chrono::steady_clock::now();
                while (std::chrono::duration_cast<std::chrono::milliseconds>  // high torque should be measured for delayTime
                               (std::chrono::steady_clock::now() - firstTimeHighTorque).count() < delayTime * 1000 &&
                       exitLoop == 0) {
                    this->updateRobot(true);
                    usleep(10000);

                    if (std::abs(this->getTorque()[i]) < thresholdTorque) {  // if torque value reach below thresholdTorque, goes back
                        spdlog::debug("Torque drop", this->getTorque()[i]);
                        highTorqueReached = false;
                        break;
                    }
                }
            }
            success[i] = highTorqueReached;
        }

        if (success[i]) {
            spdlog::info("Homing Succeeded for Joint {} .", i);
            usleep(10000);
            if (i == X2_LEFT_HIP || i == X2_RIGHT_HIP) {  // if it is a hip joint

                // zeroing is done depending on the limits on the homing direction
                if (homingDirection[i] > 0)
                    ((X2Joint *)this->joints[i])->setPositionOffset(x2Parameters.jointPositionLimits.hipMax);
                else
                    ((X2Joint *)this->joints[i])->setPositionOffset(x2Parameters.jointPositionLimits.hipMin);
            } else if (i == X2_LEFT_KNEE || i == X2_RIGHT_KNEE) {  // if it is a knee joint

                // zeroing is done depending on the limits on the homing direction
                if (homingDirection[i] > 0)
                    ((X2Joint *)this->joints[i])->setPositionOffset(x2Parameters.jointPositionLimits.kneeMax);
                else
                    ((X2Joint *)this->joints[i])->setPositionOffset(x2Parameters.jointPositionLimits.kneeMin);
            }
            // fell joint down from the limit
            initTorqueControl();
            sleep(2);

        } else {
            spdlog::error("Homing Failed for Joint {} .", i);
        }
    }
    // Checking if all commanded joint successfully homed
    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        if (homingDirection[i] == 0) continue;  // skip the joint if it is not asked to do homing
        if (success[i] == false) return false;
    }
    return true;  // will come here if all joints successfully homed
}

bool X2Robot::setBackpackIMUMode(IMUOutputMode imuOutputMode) {

    if(!x2Parameters.imuParameters.useIMU) return false;

    for(int i = 0; i<numberOfIMUs_; i++){
        if(x2Parameters.imuParameters.location[i] == 'b'){
            if(!technaidIMUs->setOutputMode(i, imuOutputMode)){
                return false;
            } else return true;
        }
    }
}

bool X2Robot::setContactIMUMode(IMUOutputMode imuOutputMode) {

    for(int i = 0; i<technaidIMUs->getNumberOfIMUs_(); i++){
        if(x2Parameters.imuParameters.location[i] == 'c'){
            if(!technaidIMUs->setOutputMode(i, imuOutputMode)){
                return false;
            }
        }
    }
    return true;
}

bool X2Robot::initialiseJoints() {
    for (int id = 0; id < X2_NUM_JOINTS; id++) {
        motorDrives.push_back(new CopleyDrive(id + 1));
        // The X2 has 2 Hips and 2 Knees, by default configured as 2 hips, then 2 legs int jointID, double jointMin, double jointMax, JointDrivePairs jdp, Drive *drive
        if (id == X2_LEFT_HIP || id == X2_RIGHT_HIP) {
            joints.push_back(new X2Joint(id, x2Parameters.jointPositionLimits.hipMin, x2Parameters.jointPositionLimits.hipMax, hipJDP, motorDrives[id]));
        } else if (id == X2_LEFT_KNEE || id == X2_RIGHT_KNEE) {
            joints.push_back(new X2Joint(id, x2Parameters.jointPositionLimits.kneeMin, x2Parameters.jointPositionLimits.kneeMax, kneeJDP, motorDrives[id]));
        }
        spdlog::debug("X2Robot::initialiseJoints() loop");
    }

    return true;
}

bool X2Robot::initialiseNetwork() {
    spdlog::debug("X2Robot::initialiseNetwork()");

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }

    return true;
}

bool X2Robot::initialiseInputs() {

    // FIXME: Temporary solution to strain gauge scale factors
    constexpr std::array<double, X2_NUM_FORCE_SENSORS> scale {
        -0.019193478, -0.016627119, 0.015766071, 0.01308
    };

    for (int id = 0; id < X2_NUM_FORCE_SENSORS; id++) {
        forceSensors.push_back(new FourierForceSensor(id + 17, scale[id]));
        inputs.push_back(forceSensors[id]);
    }

    buttons = new FourierHandle(9);
    inputs.push_back(buttons);

    return true;
}

bool X2Robot::loadParametersFromYAML(YAML::Node params_node) {

    auto params = params_node[robotName]["ros__parameters"];

    x2Parameters.maxTorque = params["max_torque"].as<double>();
    x2Parameters.maxVelocity = params["max_velocity"].as<double>();
    x2Parameters.jointPositionLimits.hipMax = deg2rad(params["position_limits"]["max_hip"].as<double>());
    x2Parameters.jointPositionLimits.hipMin = deg2rad(params["position_limits"]["min_hip"].as<double>());
    x2Parameters.jointPositionLimits.kneeMax = deg2rad(params["position_limits"]["max_knee"].as<double>());
    x2Parameters.jointPositionLimits.kneeMin = deg2rad(params["position_limits"]["min_knee"].as<double>());

    _StrainGauge.set_coeff_a(
        params["strain_gauge"]["coeff_a"].as<std::array<double, STRAIN_GAUGE_FILTER_ORDER + 1>>()
    );
    _StrainGauge.set_coeff_b(
        params["strain_gauge"]["coeff_b"].as<std::array<double, STRAIN_GAUGE_FILTER_ORDER + 1>>()
    );

    return true;
}

void X2Robot::freeMemory() {
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    for (auto p : motorDrives) {
        spdlog::debug("Delete Drive Node: {}", p->getNodeID());
        delete p;
    }
    for (auto p : inputs) {
        spdlog::debug("Deleting Input");
        delete p;
    }
}

void X2Robot::updateRobot(bool duringHoming) {
#ifndef SIM
    Robot::updateRobot();
    updateBackpackAndContactAnglesOnMedianPlane();
    updateBackpackAngularVelocity();
    updateForceMeasurements();
    updateGeneralizedAcceleration();
#endif
    // updateDynamicTerms();
    // updateInteractionForce();
    // updateFeedforwardTorque();

    _StrainGauge.filter(jointTorquesViaStrainGauges_);
}

bool X2Robot::safetyCheck(bool duringHoming) {

    for (int jointId = 0; jointId<X2_NUM_JOINTS; jointId++){
        if(abs(getVelocity()[jointId]) >= x2Parameters.maxVelocity){
            spdlog::critical("Maximim velocity limit ({}) is achieved for joint {}", getVelocity()[jointId], jointId);
            return false;
        }
        if(duringHoming) continue; // do not check torque limit during homing
        if(abs(getTorque()[jointId]) >= x2Parameters.maxTorque){
            spdlog::critical("Maximim torque limit ({}) is achieved for joint {}", getTorque()[jointId], jointId);
            return false;
        }
    }
    return true;
}

bool X2Robot::setPosControlContinuousProfile(bool continuous){
    bool returnValue = true;
    for (auto p : joints) {
        if(!(p->setPosControlContinuousProfile(continuous))){
            returnValue = false;
        }
    }
    return returnValue;
}

void X2Robot::updateDynamicTerms() {

    double gAcc = 9.81;
    double m_1 = x2Parameters.mBackpack;
    double m_2 = x2Parameters.mThigh;
    double m_3 = x2Parameters.mShank;
    double m_4 = x2Parameters.mThigh;
    double m_5 = x2Parameters.mShank;
    double lx =  x2Parameters.s[6];
    double l_1 = x2Parameters.s[5];
    double l_2 = x2Parameters.l[0];
    double l_3 = x2Parameters.l[1];
    double l_4 = x2Parameters.l[2];
    double l_5 = x2Parameters.l[3];
    double h_1 = 0.0;
    double h_2 = x2Parameters.sThighLeft;
    double h_3 = x2Parameters.sShankLeft;
    double h_4 = x2Parameters.sThighRight;
    double h_5 = x2Parameters.sShankRight;
    double I_1 = x2Parameters.LBackpack;
    double I_2 = x2Parameters.LThighLeft;
    double I_3 = x2Parameters.LShankLeft;
    double I_4 = x2Parameters.LThighRight;
    double I_5 = x2Parameters.LShankRight;
    double th_b = getBackPackAngleOnMedianPlane();
    double th_1 = getPosition()[0];
    double th_2 = getPosition()[1];
    double th_3 = getPosition()[2];
    double th_4 = getPosition()[3];

    // Dynamic parameters are obtained from the x2HybridModeling repo
    // This is a simplified model, where coupling between two legs and the effect of backpack is not considered.
    gravitationTorque_(1) = m_3*(l_2*cos(th_1+th_b)+h_3*cos(th_1+th_2+th_b))*(-9.81E+2/1.0E+2)-h_2*m_2*cos(th_1+th_b)*(9.81E+2/1.0E+2);
    gravitationTorque_(2) = h_3*m_3*cos(th_1+th_2+th_b)*(-9.81E+2/1.0E+2);
    gravitationTorque_(3) = m_5*(l_4*cos(th_3+th_b)+h_5*cos(th_3+th_4+th_b))*(-9.81E+2/1.0E+2)-h_4*m_4*cos(th_3+th_b)*(9.81E+2/1.0E+2);
    gravitationTorque_(4) = h_5*m_5*cos(th_3+th_4+th_b)*(-9.81E+2/1.0E+2);

    massMatrix_(1, 1) = x2Parameters.G[0] + I_2+I_3+(h_2*h_2)*m_2+(h_3*h_3)*m_3+(l_2*l_2)*m_3+h_3*l_2*m_3*cos(th_2)*2.0;
    massMatrix_(1, 2) = I_3+(h_3*h_3)*m_3+h_3*l_2*m_3*cos(th_2);
    massMatrix_(2, 1) = I_3+(h_3*h_3)*m_3+h_3*l_2*m_3*cos(th_2);
    massMatrix_(2, 2) = x2Parameters.G[1] + I_3+(h_3*h_3)*m_3;
    massMatrix_(3, 3) = x2Parameters.G[2] + I_4+I_5+(h_4*h_4)*m_4+(h_5*h_5)*m_5+(l_4*l_4)*m_5+h_5*l_4*m_5*cos(th_4)*2.0;
    massMatrix_(3, 4) = I_5+(h_5*h_5)*m_5+h_5*l_4*m_5*cos(th_4);
    massMatrix_(4, 3) = I_5+(h_5*h_5)*m_5+h_5*l_4*m_5*cos(th_4);
    massMatrix_(4, 4) = x2Parameters.G[3] + I_5+(h_5*h_5)*m_5;

}

void X2Robot::updateFrictionTorque(Eigen::VectorXd motionIntend) {
    const float velTreshold = 1.0*M_PI/180.0; // [rad/s]

    for(int i = 0; i<X2_NUM_JOINTS; i++){
        if(abs(getVelocity()[i]) > velTreshold){ // if in motion
            frictionTorque_[i + 1] = x2Parameters.c1[i]*getVelocity()[i]/abs(getVelocity()[i]) +
                                     x2Parameters.c0[i]*getVelocity()[i];
        }else { // if static
            frictionTorque_[i + 1] = x2Parameters.c1[i]*motionIntend[i+1]/abs(motionIntend[i+1]);
        }
    }
}

void X2Robot::updateFeedforwardTorque() {
    Eigen::VectorXd motionIntend(X2_NUM_GENERALIZED_COORDINATES);
    for(int id = 0; id <X2_NUM_GENERALIZED_COORDINATES; id++) {
        motionIntend[id] = this->getInteractionForce()[id] > 0 ? 1 : -1;
    }
    updateFrictionTorque(motionIntend);
    feedForwardTorque_ = gravitationTorque_ + corriolisTorque_ + frictionTorque_;
}

Eigen::VectorXd &X2Robot::getPosition() {
#ifndef NOROBOT
    return Robot::getPosition();
#else
    return simJointPositions_;
#endif
}

Eigen::VectorXd &X2Robot::getVelocity() {
#ifndef NOROBOT
    return Robot::getVelocity();
#else
    return simJointVelocities_;
#endif
}

Eigen::VectorXd &X2Robot::getTorque() {
#ifndef NOROBOT
    return Robot::getTorque();
#else
    return simJointTorques_;
#endif
}

Eigen::VectorXd &X2Robot::getJointTorquesViaStrainGauges() {
#ifndef NOROBOT
    return jointTorquesViaStrainGauges_;
#else
    return simJointTorquesViaStrainGauges_;
#endif
}

X2Robot::Butter &X2Robot::getStrainGaugeFilter() {
    return _StrainGauge;
}

Eigen::Vector4d X2Robot::getStrainGauges() {
    return _StrainGauge.output();
}

Eigen::VectorXd &X2Robot::getInteractionForce() {
#ifndef NOROBOT
    return interactionForces_;
#else
    return simInteractionForces_; // TODO: add force sensor to simulation
#endif
}

Eigen::VectorXd & X2Robot::getSmoothedInteractionForce() {
    return smoothedInteractionForces_;
}

Eigen::VectorXd &X2Robot::getGroundReactionForces() {
#ifndef NOROBOT
    return groundReactionForces_;
#else
    return simGroundReactionForces_; // TODO: add grf sensor to simulation
#endif

}

Eigen::VectorXd & X2Robot::getEstimatedGeneralizedAcceleration() {
    return estimatedGeneralizedAcceleration_;
}

double & X2Robot::getBackPackAngleOnMedianPlane() {
#ifndef NOROBOT
    return backPackAngleOnMedianPlane_;
#else
    return simBackPackAngleOnMedianPlane_; // TODO: add IMU to simulati
#endif
}

double & X2Robot::getBackPackAngularVelocityOnMedianPlane() {
#ifndef NOROBOT
    return backPackAngularVelocityOnMedianPlane_;
#else
    return simBackPackAngularVelocityOnMedianPlane_; // TODO: add IMU to simulati
#endif

}

Eigen::VectorXd & X2Robot::getContactAnglesOnMedianPlane() {

#ifndef NOROBOT
    return contactAnglesOnMedianPlane_;
#else
    return simContactAnglesOnMedianPlane_; // TODO: add IMU to simulati
#endif

}

Eigen::MatrixXd & X2Robot::getMassMatrix() {
    return massMatrix_;
}

Eigen::MatrixXd & X2Robot::getSelectionMatrix() {
    return selectionMatrix_;
}

Eigen::VectorXd & X2Robot::getGravitationTorque() {
    return gravitationTorque_;
}

Eigen::VectorXd & X2Robot::getCorriolisTorque() {
    return corriolisTorque_;
}

Eigen::VectorXd & X2Robot::getFrictionTorque() {
    return frictionTorque_;
}

Eigen::VectorXd & X2Robot::getFeedForwardTorque() {
    return  feedForwardTorque_;
}

double & X2Robot::getButtonValue(ButtonColor buttonColor) {
    return buttons->getButtonValues()[buttonColor];
}

Eigen::MatrixXd X2Robot::getPseudoInverseOfSelectionMatrixTranspose() {
    return pseudoInverseOfSelectionMatrixTranspose_;
}

ControlMode & X2Robot::getControlMode() {
    return controlMode;
}

void X2Robot::setRobotName(std::string robotName) {
    robotName = robotName;
}

std::string & X2Robot::getRobotName() {
    return robotName;
}

RobotParameters& X2Robot::getRobotParameters() {
    return x2Parameters;
}

Eigen::VectorXd & X2Robot::getGRFSensorThresholds() {
    return x2Parameters.grfSensorThreshold;
}

double & X2Robot::getJointVelDerivativeCutOffFrequency() {
    return jointVelDerivativeCutOffFreq_;
}

double & X2Robot::getBackpackVelDerivativeCutOffFrequency() {
    return backpackVelDerivativeCutOffFreq_;
}

double & X2Robot::getDynamicParametersCutOffFrequency() {
    return dynamicParametersCutOffFreq_;
}

void X2Robot::setJointVelDerivativeCutOffFrequency(double cutOffFrequency) {
    jointVelDerivativeCutOffFreq_ = cutOffFrequency;
}

void X2Robot::setBackpackVelDerivativeCutOffFrequency(double cutOffFrequency) {
    backpackVelDerivativeCutOffFreq_ = cutOffFrequency;
}

void X2Robot::setDynamicParametersCutOffFrequency(double cutOffFrequency) {
    dynamicParametersCutOffFreq_ = cutOffFrequency;
}

void X2Robot::setGRFSensorsThreshold(Eigen::VectorXd thresholds) {

    //TODO: ADD A FOR LOOP
    x2Parameters.grfSensorThreshold[0] = thresholds[0];
    x2Parameters.grfSensorThreshold[1] = thresholds[1];
}

#ifdef SIM
void X2Robot::setNodeHandle(ros::NodeHandle &nodeHandle) {
    nodeHandle_ = &nodeHandle;
}

void X2Robot::jointStateCallback(const sensor_msgs::JointState &msg) {
    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        simJointPositions_[i] = msg.position[i];
        simJointVelocities_[i] = msg.velocity[i];
        simJointTorques_[i] = msg.effort[i];
    }
}

#endif
