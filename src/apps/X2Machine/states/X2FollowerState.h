#ifndef SRC_X2FOLLOWERSTATE_H
#define SRC_X2FOLLOWERSTATE_H

#include "State.h"
#include "X2Robot.h"
#include "controller.hpp"
#include "LookupTable.hpp"
#include "kinematics.hpp"
#include <map>



#define LIMIT_TORQUE    80 // [Nm]

#define LEFT_HIP        0
#define LEFT_KNEE       1
#define RIGHT_HIP       2
#define RIGHT_KNEE      3

#define IK              0
#define GAIT            1
#define IK_GAIT         2
#define IDLE            3
#define TUNE            4
#define TEST            5

#define STEP_UP         0
#define STEP_DOWN       1

class X2FollowerState : public State {
    X2Robot* robot_;

public:
    double maxTorqueLimit;
    double rateLimit;
    int refPosPeriod;
    Eigen::VectorXd debugTorques;
    Eigen::VectorXd frictionCompensationTorques;

    Eigen::VectorXd desiredJointReferences_; //Used to communicate with the IK node
    //Storage variables for robot qualtities
    Eigen::VectorXd jointTorques_;
    Eigen::VectorXd jointPositions_;
    Eigen::VectorXd prevJointPositions_;
    Eigen::VectorXd prevJointReferences_;
    Eigen::VectorXd dJointPositions_; //Numerical derivative of the jointPositions
    Eigen::VectorXd dJointReferences_;

    std::string csvFileName;
    Eigen::VectorXd& getDesiredJointTorques();
    Eigen::VectorXd& getDesiredJointPositions();
    Eigen::VectorXd& getDesiredJointTorquesPSplit();
    Eigen::VectorXd& getDesiredJointTorquesISplit();
    Eigen::VectorXd& getDesiredJointTorquesDSplit();
    Eigen::VectorXd& getDesiredJointVelocities();
    Eigen::VectorXd& getActualDesiredJointPositions();


    void entry(void);
    void during(void);
    void exit(void);
    X2FollowerState(StateMachine* m, X2Robot* exo, const float updateT, const char* name = NULL);
    
    timespec origTime;
    timespec currTime;
    std::ofstream PDFile;
    
    // Logging Functions
    std::chrono::time_point<std::chrono::steady_clock> _Time_prev; // Used to identify Derivative.
    std::chrono::time_point<std::chrono::steady_clock> _Time; // Used to identify Derivative.
    std::shared_ptr<spdlog::logger> dpos_logger;
    std::shared_ptr<spdlog::logger> pos_logger;
    std::shared_ptr<spdlog::logger> dref_logger;
    std::shared_ptr<spdlog::logger> ref_logger;
    std::shared_ptr<spdlog::logger> p_logger; //P Component of the output
    std::shared_ptr<spdlog::logger> d_logger; //D Component of the output
    std::shared_ptr<spdlog::logger> torque_logger; //Torque commanded by PD controller
    std::shared_ptr<spdlog::logger> effort_logger; //Copely measured torque
    std::shared_ptr<spdlog::logger> err_logger; //The error used to compute  P and D component
    
    


    enum cntrl {PD, Ext, Fric, Grav};
    ctrl::PDController<double, X2_NUM_JOINTS, 500>* PDCntrl;
    ctrl::ExternalController<double, X2_NUM_JOINTS>* ExtCntrl;
    ctrl::FrictionController<double, X2_NUM_JOINTS>* FricCntrl;
    ctrl::GravityController<double, X2_NUM_JOINTS>* GravCntrl;
    std::array<ctrl::BaseController<double, X2_NUM_JOINTS>*, 4> controllers;

    bool checkSafety(void);
    
    LookupTable<double, X2_NUM_JOINTS> posReader;

private:
    const int freq_;
    int t_count_;
    int state_;


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