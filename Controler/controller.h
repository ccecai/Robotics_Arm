//
// Created by 1 on 2024-05-02.
//

#ifndef ROBOTIC_ARM_B_CONTROLLER_H
#define ROBOTIC_ARM_B_CONTROLLER_H

#include "math.h"
#include "main.h"

#define L1 15 //cm
#define L2 15 //cm
#define radius 7.5 //cm
#define PI 3.1415926535

extern float theta1,theta2,theta3;
extern float px,py,pz;

void DH_init(void);
void KF_kinematics(float theta1,float theta2,float theta3);
void IF_kinematics(float x,float y,float z);
float angle_to_radian(float angle);
float radian_to_angle(float radian);

#endif //ROBOTIC_ARM_B_CONTROLLER_H
