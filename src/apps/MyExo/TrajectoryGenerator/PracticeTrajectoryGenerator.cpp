/**
 * @file PracticeTrajectoryGenerator.cpp
 * @author Joshua (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "PracticeTrajectoryGenerator.hpp"

double sitting[6] = {deg2rad(95), deg2rad(95), deg2rad(90), deg2rad(90), 0, 0};
double standing[6] = {0, 0, 0, 0, 0, 0};

Eigen::VectorXd startPos(6);

PracticeTrajectoryGenerator::PracticeTrajectoryGenerator(int NumOfJoints) 
{
    double sitPos[6] = {deg2rad(95),deg2rad(95), deg2rad(90), deg2rad(90),0,0};
    double standPos[6] =  {0, 0, 0, 0, 0, 0};

    numJoints = NumOfJoints;
    for(int i = 0; i < NumOfJoints; i++) {
        sitting[i] = sitPos[i];
        standing[i] = standPos[i];
    }
}

bool PracticeTrajectoryGenerator::initialiseTrajectory() 
{
    currentTraj = SIT;
    trajTime = 2;
    lastProgress = 0;
    return true;
}

bool PracticeTrajectoryGenerator::initialiseTrajectory(Trajectory traj, double time, Eigen::VectorXd &startPos_) 
{
    currentTraj = traj;
    trajTime = time;
    startPos = startPos_;
    lastProgress = 0;
    return true;
}

Eigen::VectorXd  PracticeTrajectoryGenerator::getSetPoint(double time) {
    double progress = time / trajTime;
    Eigen::VectorXd angles(numJoints);

    if (currentTraj == SIT) {
        for (int i = 0; i < numJoints; i++) {

            if (progress > 1) {
                angles(i) = sitting[i];
            } else {
                angles(i) = startPos[i] + progress * (sitting[i] - startPos[i]);
            }
        }
    } else {
        for (int i = 0; i < numJoints; i++) {
            if (progress > 1) {
                angles(i) = standing[i];
            } else {
                angles(i) = startPos[i]  + progress * (standing[i] - startPos[i]);
            }
        }
    }
    lastProgress = progress;

    return angles;
}

bool PracticeTrajectoryGenerator::isTrajcetoryFinished() {
    if (lastProgress > 1.0) {
        return true;
    } else {
        return false;
    }
}