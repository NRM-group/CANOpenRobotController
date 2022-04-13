/**
 * @file StandState.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "StandState.h"

void StandState::entry(void)
{
    spdlog::info("Stand State Entered");
    std::cout << "Press the S button to sit" << std::endl;
}

void StandState::during(void) {}

void StandState::exit(void) 
{
    spdlog::info("Standing state exited"); 
}