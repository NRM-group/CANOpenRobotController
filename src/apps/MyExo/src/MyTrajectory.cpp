#include "MyTrajectory.h"

MyTrajectory::MyTrajectory(int jointCount) : m_joint_count(jointCount)
{
}

bool
MyTrajectory::initialise_trajectory(void)
{
    m_current_traj = SIT;
    m_traj_time = 2;
    m_last_progress = 0;
    return true;
}

bool
MyTrajectory::initialise_trajectory(
        Trajectory traj,
        double time,
        Eigen::VectorXd& startPos)
{
    m_current_traj = traj;
    m_traj_time = time;
    m_last_progress = 0;
    START_POS = startPos;
    return true;
}

Eigen::VectorXd
MyTrajectory::get_set_point(double time)
{
    double progress = time / m_traj_time;
    Eigen::VectorXd angles(m_joint_count);

    if (m_current_traj == SIT) {
        for (int i = 0; i < m_joint_count; i++) {
            if (progress > 1) {
                angles(i) = SITTING[i];
            } else {
                angles(i) = START_POS[i] + progress *
                        (SITTING[i] - START_POS[i]);
            }
        }
    } else {
        for (int i = 0; i < m_joint_count; i++) {
            if (progress > 1) {
                angles(i) = STANDING[i];
            } else {
                angles(i) = START_POS[i] + progress *
                        (STANDING[i] - START_POS[i]);
            }
        }
    }
    m_last_progress = progress;
    return angles;
}

bool
MyTrajectory::is_trajectory_finished(void)
{
    return m_last_progress > 1.0;
}
