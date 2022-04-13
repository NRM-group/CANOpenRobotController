#ifndef __MY_TRAJECTORY_H
#define __MY_TRAJECTORY_H

#include <cmath>
#include <time.h>
#include <vector>

#include <Eigen/Dense>

#include "LogHelper.h"

#define DEG2RAD(deg)    ((deg) * M_PI / 180.0)
#define RAD2DEG(rad)    ((rad) * 180.0 / M_PI)

static const double SITTING[6] = {
    DEG2RAD(95), DEG2RAD(95), DEG2RAD(90), DEG2RAD(90), 0, 0
};

static const double STANDING[6] = {
    0, 0, 0, 0, 0, 0
};
enum Trajectory {
        LEFT,
        RIGHT,
        SIT,
        STAND
    };


static Eigen::VectorXd START_POS(6);

class MyTrajectory
{
    
public:
    MyTrajectory(int jointCount);
    bool initialise_trajectory(void);
    bool initialise_trajectory(
            Trajectory traj,
            double time,
            Eigen::VectorXd& startPos
    );
    Eigen::VectorXd get_set_point(double time);
    bool is_trajectory_finished(void);

private:
    std::vector<double[2]>  m_end_points;
    Trajectory              m_current_traj;
    double                  m_traj_time;
    double                  m_last_progress;
    double                  m_current_trajectory_progress;
    int                     m_joint_count;
    timespec                m_prev_time;

};

#endif//__MY_TRAJECTORY_H
