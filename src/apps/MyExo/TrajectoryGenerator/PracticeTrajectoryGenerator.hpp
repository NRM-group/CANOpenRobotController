/**
 * @file PracticeTrajectoryGenerator.hpp
 * @author Joshua Lai
 * @brief 
 * @version 0.1
 * @date 2022-03-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef PRACTICETRAJECTORYGENERATOR_H_INCLUDED
#define PRACTICETRAJECTORYGENERATOR_H_INCLUDED
#include <time.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "LogHelper.h"

#define deg2rad(deg) ((deg) * M_PI / 180.0)
#define rad2deg(rad) ((rad) * 180.0 / M_PI)


enum Trajectory {
    SIT = 0,
    STAND = 1,
};

/**
 * @brief Taken from DummyTrajectoryGenerator.h
 * An Example of the Trajectory Generator
 */

class PracticeTrajectoryGenerator 
{
    private:

    std::vector<double[2]> endPoints;
    Trajectory currentTraj = SIT;
    double trajTime = 2;
    int numJoints = 4;
    double lastProgress = 0;
    double currentTrajProgress = 0;
    timespec prevTime;
    
    public:
    PracticeTrajectoryGenerator(int NumOfJoints);
    /**
     * @brief Initialise Trajectory function
     * 
     */
    bool initialiseTrajectory();

    bool initialiseTrajectory(Trajectory traj, double time, Eigen::VectorXd &startPos_);

    Eigen::VectorXd  getSetPoint(double time);

    bool isTrajcetoryFinished();

};
#endif
