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
extern uint8_t Pro_flag;

#define Screen_Length 10
#define IF_MODE 1
#define Demonstration_MODE 2
#define Gravity_compensation_MODE 3

void Screen_DataProcess(void);

#endif //ROBOTICS_ARM_SCREEN_H
