//
// Created by 1 on 2024-05-04.
//

#ifndef ROBOTICS_ARM_SCREEN_H
#define ROBOTICS_ARM_SCREEN_H

#include "main.h"
#include "controller.h"
#include "DeepMotor.h"
#include "Programming_by_Demonstration.h"

extern uint8_t Screen_Data[10];

#define Screen_Length 10

void Screen_DataProcess(void);

#endif //ROBOTICS_ARM_SCREEN_H
