/**
 * @file gaitCycle.hpp
 * @author Nilp Amin
 * @brief 
 * @version 0.1
 * @date 2022-04-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __GAIT_CYCLE_HPP
#define __GAIT_CYCLE_HPP
#include <time.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "LogHelper.h"

#define DEG2RAD(deg)    ((deg * M_PI / 180.0))
#define RAD2DEG(rad)    ((rad) * 180.0 M_PI)

class GaitCycleLookUp
{
public:
    
    /**
     * @brief Construct a new Gait Cycle Look Up object
     * 
     */
    GaitCycleLookUp(void);

private:
};

#endif