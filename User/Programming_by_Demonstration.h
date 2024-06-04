//
// Created by 1 on 2024-05-04.
//

#ifndef ROBOTICS_ARM_PROGRAMMING_BY_DEMONSTRATION_H
#define ROBOTICS_ARM_PROGRAMMING_BY_DEMONSTRATION_H

#include "main.h"
#include "controller.h"
#include "DeepMotor.h"
#include "math.h"
#include "Screen.h"

#define LOW_SPEED 0.7f
#define HIGH_SPEED 1.2f

extern float Memory_theta[7][50];
extern float speed_kd;
extern float time;
extern uint8_t count_flag;
extern float times[50];
extern uint8_t count_t;

void Demonstration(void);
void by_Programming(void );
float ABS(float a);

#endif //ROBOTICS_ARM_PROGRAMMING_BY_DEMONSTRATION_H
