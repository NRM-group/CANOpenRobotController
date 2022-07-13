#ifndef SRC_X2FOLLOWERSTATE_H
#define SRC_X2FOLLOWERSTATE_H

#include "State.h"
#include "X2Robot.h"
#include "controller.hpp"
#include "LookupTable.h"
#include "kinematics.hpp"
#include <map>

#define VAR_LOG(x)      spdlog::info("{} {} {} {}", x[0], x[1], x[2], x[3])
#define CTRL_LOG(x)     spdlog::info("{} {} {} {}", x.output()[0], x.output()[1], x.output()[2], x.output()[3])

#define LIMIT_TORQUE    80 // [Nm]

#define LEFT_HIP        0
#define LEFT_KNEE       1
#define RIGHT_HIP       2
#define RIGHT_KNEE      3

#define IK  0
#define GAIT    1
#define IK_GAIT 2

class X2FollowerState : public State {
    X2Robot* robot_;

public:
    double maxTorqueLimit;
    double rateLimit;
    double refPos1;
    double refPos2;
    int refPosPeriod;
    Eigen::VectorXd debugTorques;
    Eigen::VectorXd frictionCompensationTorques;

    Eigen::VectorXd desiredJointReferences_; //Used to communicate with the IK node

    std::string csvFileName;
    Eigen::VectorXd& getDesiredJointTorques();
    Eigen::VectorXd& getDesiredJointPositions();
    Eigen::VectorXd& getDesiredJointTorquesPSplit();
    Eigen::VectorXd& getDesiredJointTorquesISplit();
    Eigen::VectorXd& getDesiredJointTorquesDSplit();
    Eigen::VectorXd& getDesiredJointVelocities();
    Eigen::VectorXd& getActualDesiredJointPositions();

    double strainGauge_[X2_NUM_JOINTS];
    double filteredGauge_[X2_NUM_JOINTS];
    ctrl::Butterworth<double, X2_NUM_JOINTS, 5> butter;

    void entry(void);
    void during(void);
    void exit(void);
    X2FollowerState(StateMachine* m, X2Robot* exo, const float updateT, const char* name = NULL);
    
    ctrl::FrictionController<double, X2_NUM_JOINTS> friction;
    ctrl::GravityController<double, X2_NUM_JOINTS> gravity;
    ctrl::TorqueController<double, X2_NUM_JOINTS> torque;

    bool checkSafety(void);
    
    LookupTable posReader;

private:
    const int freq_;
    int t_count_;


    timespec prevTime;
    int gaitIndex;
    double trajTime;
    double currTrajProgress;

    bool safetyFlag;

    int mode;
    std::chrono::steady_clock::time_point time0;
    Eigen::VectorXd desiredJointPositions_;         // the desired joint positions
    Eigen::VectorXd actualDesiredJointPositions_;
    Eigen::VectorXd prevDesiredJointPositions_;     // the previous desired joint position set by rate limiter
    Eigen::VectorXd desiredJointVelocities_;
    Eigen::VectorXd desiredJointTorques_;
    Eigen::VectorXd prevDesiredJointTorques_;
    Eigen::VectorXd desiredJointTorquesP_;
    Eigen::VectorXd desiredJointTorquesI_;
    Eigen::VectorXd desiredJointTorquesD_;
    Eigen::VectorXd startJointPositions_;



    void rateLimiter(double limit);
    void torqueLimiter(double limit);
    void addDebugTorques(int joint);
    void addFrictionCompensationTorques(int joint);
    
    LegKinematics<double> kinHandler;
};

#endif