#include "GaitTestState.h"

void GaitTestState::entry(void) {
    spdlog::info("Entered Gait testing State");
    std::cout 
        << "======================" << std::endl
        << " HIT W -> Stand UP" << std::endl
        << "======================" << std::endl;
    //Initialise the csv intepreter
    csvReader = LookupTable(4);
    clock_gettime(CLOCK_MONOTONIC, &prevTime);
    gaitIndex = 0;
}

void GaitTestState::during(void) {
    timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);
    double elapsedSec = currTime.tv_sec - prevTime.tv_sec + (currTime.tv_nsec - prevTime.tv_nsec) / 1e9;
    Eigen::VectorXd joints(4);
    if (elapsedSec >= PERIOD) {
        if (gaitIndex == 100) {
            gaitIndex = 0;
        }
        for (int jointNo = 0; jointNo < 4; jointNo ++ ){
            joints(jointNo) = csvReader.getPosition(jointNo, gaitIndex);
        }
        robot->setPosition(joints);
        prevTime = currTime;
        gaitIndex ++;
    }
}

void GaitTestState::exit(void) {
    spdlog::info("Exiting Gait Testing state ");

}