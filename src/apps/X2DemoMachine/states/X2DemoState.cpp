#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, const char *name) :
        State(m, name), robot_(exo) {
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

    startJointPositions_ = robot_->getPosition();
    currTrajProgress = 0;

    enableJoints = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    kTransperancy_ = Eigen::VectorXd::Zero(X2_NUM_GENERALIZED_COORDINATES);
    amplitude_ = 0.0;
    period_ = 5.0;
    offset_ = 0.0;

}

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    // set dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&X2DemoState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);
    //Simulation has a length of 360mm
    legkin.update_lengths(360, 360, LEFT);
    legkin.update_lengths(360, 360, RIGHT);
    desiredCartesianPosition(0) = 680;
    desiredCartesianPosition(1) = 0;
    posReader = LookupTable<double, X2_NUM_JOINTS>();
    posReader.readCSV("/home/fred-ross/catkin_ws/src/CANOpenRobotController/lib/trajectorylib/gaits/walking.csv");
    posReader.startTrajectory(robot_->getPosition());
    time0 = std::chrono::steady_clock::now();

}

void X2DemoState::during(void) {

#ifndef SIM
    // GREEN BUTTON IS THE DEAD MAN SWITCH --> if it is not pressed, all motor torques are set to 0. Except controller 2 which sets 0 velocity
    // if(robot_->getButtonValue(ButtonColor::GREEN) == 0 && controller_mode_ !=2){
    //     if(robot_->getControlMode()!=CM_TORQUE_CONTROL) robot_->initTorqueControl();
    //     desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    //     robot_->setTorque(desiredJointTorques_);

    //     return;
    // }
#endif
    
    if(controller_mode_ == 1){ // zero torque mode

        if(robot_->getControlMode()!=CM_TORQUE_CONTROL){
            robot_->initTorqueControl();
        }

        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);

    } else if(controller_mode_ == 2){ // IK_testing
        if (robot_->getControlMode()!=CM_POSITION_CONTROL) robot_->initPositionControl();
        double x = desiredCartesianPosition(0);
        double y = desiredCartesianPosition(1);
        Eigen::VectorXd legCoords(4);
        legCoords << x,y,x,y;
        Eigen::Matrix<double, 2,1> angles = legkin.inv_kin(legCoords);
        Eigen::VectorXd destination = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        timespec currTime;
        clock_gettime(CLOCK_MONOTONIC, &currTime);

        
        double timeElapsed = currTime.tv_sec - prevTime.tv_sec + (currTime.tv_nsec - prevTime.tv_nsec) / 1e9;
        prevTime = currTime;
        currTrajProgress += timeElapsed; 
        double progress = currTrajProgress / trajTime;
        trajTime = period_;
        if(progress >= 1) {
            //When you have finished this linear point, move on to the next stage
            currTrajProgress = 0;
            startJointPositions_ = robot_->getPosition();   
            return;
        }
        destination[0] = angles(0);
        destination[1] = angles(1);
        destination[2] = angles(0);
        destination[3] = angles(1);
        //Interpolate the required changes to get to a location
        for(int j = 0; j < X2_NUM_JOINTS ; j ++) {
            desiredJointPositions_[j] = startJointPositions_[j]  + progress * (destination[j] - startJointPositions_[j]);
            if (j == JOINT_1 || j == JOINT_3) {
                // check hip bounds
                if (desiredJointPositions_(j) > deg2rad(120)) {
                    desiredJointPositions_(j) = deg2rad(120);
                } else if (desiredJointPositions_(j) < -deg2rad(40)) {
                    desiredJointPositions_(j) = -deg2rad(40);
                }
            } else if (j == JOINT_2 || j == JOINT_4) {
                // check knee bounds
                if (desiredJointPositions_(j) < -deg2rad(120)) {
                    desiredJointPositions_(j) = -deg2rad(120);
                } else if (desiredJointPositions_(j) > 0) {
                    desiredJointPositions_(j) = 0;
                }
            }
        }

        robot_->setPosition(desiredJointPositions_);
        //Increment the position of the legs to meet the deisired joint positions


    } else if(controller_mode_ == 3){ //Follwer State
        if(robot_->getControlMode() != CM_POSITION_CONTROL) robot_->initPositionControl();
        desiredJointPositions_ = posReader.getNextPos();
        robot_->setPosition(desiredJointPositions_);
        
    } else if(controller_mode_ == 4){ // sin vel
        if(robot_->getControlMode()!=CM_VELOCITY_CONTROL) robot_->initVelocityControl();

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        for(int joint = 0; joint < X2_NUM_JOINTS; joint++)
        {
        desiredJointVelocities_[joint] = enableJoints[joint]*amplitude_*sin(2.0*M_PI/period_*time);
        }

        robot_->setVelocity(desiredJointVelocities_);

    } else if(controller_mode_ == 5){ // sin torque
        if(robot_->getControlMode()!=CM_TORQUE_CONTROL) robot_->initTorqueControl();

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        for(int joint = 0; joint < X2_NUM_JOINTS; joint++)
        {
            desiredJointTorques_[joint] = enableJoints[joint]*amplitude_*sin(2.0*M_PI/period_*time);
        }
        robot_->setTorque(desiredJointTorques_);

    }
}

void X2DemoState::exit(void) {
    robot_->initTorqueControl();
    // setting 0 torque for safety.
    robot_->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
    std::cout << "Example State Exited" << std::endl;
}

void X2DemoState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {

    controller_mode_ = config.controller_mode;

    enableJoints[0] = config.left_hip;
    enableJoints[1] = config.left_knee;
    enableJoints[2] = config.right_hip;
    enableJoints[3] = config.right_knee;

    kTransperancy_[1] = config.k_left_hip;
    kTransperancy_[2] = config.k_left_knee;
    kTransperancy_[3] = config.k_right_hip;
    kTransperancy_[4] = config.k_right_knee;

    amplitude_ = config.Amplitude;
    period_ = config.Period;
    offset_ = config.offset;

    robot_->setJointVelDerivativeCutOffFrequency(config.acc_deriv_cutoff);
    robot_->setBackpackVelDerivativeCutOffFrequency(config.backpack_deriv_cutoff);
    robot_->setDynamicParametersCutOffFrequency(config.g_cutoff);

    if(controller_mode_ == 4 || controller_mode_ == 5) time0 = std::chrono::steady_clock::now();

    return;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

Eigen::VectorXd & X2DemoState::getDesiredJointVelocities() {
    return desiredJointVelocities_;
}