//
// Created by 1 on 2024-05-02.
//
#include "controller.h"
#include "printf.h"
#include "DeepMotor.h"

double theta1 = 0,theta2 = 0,theta3 = 0; // 目前所有角度均为弧度制
float px = 0,py = 0,pz = 0;
/**********
 *
 * 初始状态根据模型得到的DH参数表：
 *  i       cta            d            a         af
 * 0-1       0         2*Radius     2*Radius     90
 * 1-2       0       2*Radius+l1  2*Radius+l1   -180
 * 2-3       0         Radius+l2    Radius+l2     0
 *
 *
*/

//其中T01*T12*T23=T03 连杆的通用变化矩阵的初始化
float T01[4][4];
float T12[4][4];
float T23[4][4];
float T02[4][4];
float T03[4][4];

void DH_init(void)
{
    T01[0][0] = 1;
    T01[0][1] = 0;
    T01[0][2] = 0;
    T01[0][3] = 2 * radius;

    T01[1][0] = 0;
    T01[1][1] = 0;
    T01[1][2] = -1;
    T01[1][3] = -2 * radius;

    T01[2][0] = 0;
    T01[2][1] = 1;
    T01[2][2] = 0;
    T01[2][3] = 0;

    T01[3][0] = 0;
    T01[3][1] = 0;
    T01[3][2] = 0;
    T01[3][3] = 1;

    T12[0][0] = 1;
    T12[0][1] = 0;
    T12[0][2] = 0;
    T12[0][3] = 2 * radius + L1;

    T12[1][0] = 0;
    T12[1][1] = -1;
    T12[1][2] = 0;
    T12[1][3] = 0;

    T12[2][0] = 0;
    T12[2][1] = 0;
    T12[2][2] = -1;
    T12[2][3] = -(2 * radius + L1);

    T12[3][0] = 0;
    T12[3][1] = 0;
    T12[3][2] = 0;
    T12[3][3] = 1;

    T23[0][0] = 1;
    T23[0][1] = 0;
    T23[0][2] = 0;
    T23[0][3] = radius + L2;

    T23[1][0] = 0;
    T23[1][1] = 1;
    T23[1][2] = 0;
    T23[1][3] = 0;

    T23[2][0] = 0;
    T23[2][1] = 0;
    T23[2][2] = 1;
    T23[2][3] = radius + L2;

    T23[3][0] = 0;
    T23[3][1] = 0;
    T23[3][2] = 0;
    T23[3][3] = 1;
}

/***
* 如若开始加入旋转的角度根据模型得到的DH参数表：
*  i       cta            d            a         af
* 0-1       -theta1         2*Radius     2*Radius     90
* 1-2       -theta2       2*Radius+l1  2*Radius+l1   -180
* 2-3       -theta3         Radius+l2    Radius+l2     0
*
*/

void KF_kinematics(float theta1,float theta2,float theta3)
{

    T01[0][0] = cos(-theta1);
    T01[0][1] = -sin(theta1);
    T01[0][2] = 0;
    T01[0][3] = 2 * radius;

    T01[1][0] = sin(-theta1) * cos(angle_to_radian(90));
    T01[1][1] = cos(-theta1) * cos(angle_to_radian(90));
    T01[1][2] = -sin(angle_to_radian(90));
    T01[1][3] = -sin(angle_to_radian(90)) * 2 * radius;

    T01[2][0] = sin(-theta1) * sin(angle_to_radian(90));
    T01[2][1] = cos(-theta1) * sin(angle_to_radian(90));
    T01[2][2] = cos(angle_to_radian(90));
    T01[2][3] = cos(angle_to_radian(90)) * 2 * radius;

    T01[3][0] = 0;
    T01[3][1] = 0;
    T01[3][2] = 0;
    T01[3][3] = 1;

    T12[0][0] = cos(-theta2);
    T12[0][1] = -sin(theta2);
    T12[0][2] = 0;
    T12[0][3] = 2 * radius + L1;

    T12[1][0] = sin(-theta2) * cos(angle_to_radian(-180));
    T12[1][1] = cos(-theta2) * cos(angle_to_radian(-180));
    T12[1][2] = -sin(angle_to_radian(-180));
    T12[1][3] = -sin(angle_to_radian(-180)) * (2*radius+L1);

    T12[2][0] = sin(-theta2) * sin(angle_to_radian(-180));
    T12[2][1] = cos(-theta2) * sin(angle_to_radian(-180));
    T12[2][2] = cos(angle_to_radian(-180));
    T12[2][3] = cos(angle_to_radian(-180)) * (2*radius+L1);

    T12[3][0] = 0;
    T12[3][1] = 0;
    T12[3][2] = 0;
    T12[3][3] = 1;

    T23[0][0] = cos(-theta3);
    T23[0][1] = -sin(theta3);
    T23[0][2] = 0;
    T23[0][3] = radius + L2;

    T23[1][0] = sin(-theta3) * cos(angle_to_radian(0));
    T23[1][1] = cos(-theta3) * cos(angle_to_radian(0));
    T23[1][2] = -sin(angle_to_radian(0));
    T23[1][3] = -sin(angle_to_radian(0)) * (radius+L2);

    T23[2][0] = sin(-theta3) * sin(angle_to_radian(0));
    T23[2][1] = cos(-theta3) * sin(angle_to_radian(0));
    T23[2][2] = cos(angle_to_radian(0));
    T23[2][3] = cos(angle_to_radian(0)) * (radius+L2);

    T23[3][0] = 0;
    T23[3][1] = 0;
    T23[3][2] = 0;
    T23[3][3] = 1;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T02[i][j] = 0;
            T03[i][j] = 0;
        }

    }

    //矩阵相乘c语言转换
     for (int i = 0; i < 4; ++i) {
         for (int j = 0; j < 4; ++j) {
             for (int k = 0; k < 4; ++k) {
                 T02[i][j] += T01[i][k] * T12[k][j];
             }

         }
     }

     for (int i = 0; i < 4; ++i) {
         for (int j = 0; j < 4; ++j) {
             for (int k = 0; k < 4; ++k) {
                    T03[i][j] += T02[i][k] * T23[k][j];
             }

         }
     }

    px = T03[0][3];
    py = T03[1][3];
    pz = T03[2][3];

}

void IF_kinematics(double x,double y,double z)
{
    if(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) > (2*L1))
    {

    }
    else
    {
        if(x >= 0 && y >= 0)
        {
            theta1 = -atan(y / x);
            theta2 = -(PI / 2  - atan(z / sqrt(pow(x,2) + pow(y,2))) - acos(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) / (2*L1)));
            theta3 = acos(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) / (2*L1)) + atan(sqrt(pow(x,2) + pow(y,2)) / z);
        }
        else if(x >= 0 && y < 0)
        {
            theta1 = atan((-y) / x);
//            theta1 = -(PI / 2 + atan((-x) / y));
            theta2 = -(PI / 2  - atan(z / sqrt(pow(x,2) + pow(y,2))) - acos(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) / (2*L1)));
            theta3 = acos(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) / (2*L1)) + atan(sqrt(pow(x,2) + pow(y,2)) / z);
        }
        else if(x < 0 && y >= 0)
        {
            theta1 = -(PI / 2 + atan((-x) / y));
            theta2 = -(PI / 2  - atan(z / sqrt(pow(x,2) + pow(y,2))) - acos(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) / (2*L1)));
            theta3 = acos(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) / (2*L1)) + atan(sqrt(pow(x,2) + pow(y,2)) / z);
        }
        else if(x < 0 && y < 0)
        {
            theta1 = PI / 2 + atan((-x) / (-y));
            theta2 = -(PI / 2  - atan(z / sqrt(pow(x,2) + pow(y,2))) - acos(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) / (2*L1)));
            theta3 = acos(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) / (2*L1)) + atan(sqrt(pow(x,2) + pow(y,2)) / z);
        }

    }
}

float angle_to_radian(float angle)
{
    return angle * PI / 180;
}

float radian_to_angle(float radian)
{
    return radian * 180 / PI;
}