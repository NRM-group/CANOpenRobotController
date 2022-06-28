#ifndef SRC_X2FOLLOWERSTATE_H
#define SRC_X2FOLLOWERSTATE_H

#include "State.h"
#include "X2Robot.h"
#include "controller.hpp"
#include "LookupTable.h"
#include <map>



#define LIMIT_TORQUE    80 // [Nm]

#define LEFT_HIP        0
#define LEFT_KNEE       1
#define RIGHT_HIP       2
#define RIGHT_KNEE      3

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

    std::string csvFileName;
    Eigen::VectorXd& getDesiredJointTorques();
    Eigen::VectorXd& getDesiredJointPositions();
    Eigen::VectorXd& getDesiredJointTorquesPSplit();
    Eigen::VectorXd& getDesiredJointTorquesISplit();
    Eigen::VectorXd& getDesiredJointTorquesDSplit();
    Eigen::VectorXd& getDesiredJointVelocities();

    void entry(void);
    void during(void);
    void exit(void);
    X2FollowerState(StateMachine* m, X2Robot* exo, const float updateT, const char* name = NULL);
    

    enum cntrl {PD, Ext, Fric, Grav};
    PDController<double, X2_NUM_JOINTS>* PDCntrl;
    ExternalController<double, X2_NUM_JOINTS>* ExtCntrl;
    FrictionController<double, X2_NUM_JOINTS>* FricCntrl;
    std::array<BaseController<double, X2_NUM_JOINTS>*, 3> controllers;

    bool checkSafety(void);

private:
    const int freq_;
    int t_count_;

    LookupTable posReader;
    timespec prevTime;
    int gaitIndex;
    double trajTime;
    double currTrajProgress;

    bool safetyFlag;

    std::chrono::steady_clock::time_point time0;
    Eigen::VectorXd desiredJointPositions_;         // the desired joint positions
    Eigen::VectorXd prevDesiredJointPositions_;     // the previous desired joint position set by rate limiter
    Eigen::VectorXd desiredJointVelocities_;
    Eigen::VectorXd desiredJointTorques_;
    Eigen::VectorXd desiredJointTorquesP_;
    Eigen::VectorXd desiredJointTorquesI_;
    Eigen::VectorXd desiredJointTorquesD_;

    void rateLimiter(double limit);
    void addDebugTorques(int joint);
    void addFrictionCompensationTorques(int joint);
};

#endif