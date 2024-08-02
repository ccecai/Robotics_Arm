//
// Created by 1 on 2024-05-04.
//
#include "Programming_by_Demonstration.h"

float Memory_theta[7][50] = {0};
float speed_kd = 4.0f;
uint8_t Memory_count = 0;
float time = 0;
uint8_t count_flag = 0;
float times[50] = {0};
uint8_t count_t = 0;

void Demonstration(void)
{
    static uint8_t count = 0;

    if(ABS(Final_Data[1].Speed) > HIGH_SPEED || ABS(Final_Data[2].Speed) > HIGH_SPEED || ABS(Final_Data[3].Speed) > HIGH_SPEED
     && ABS(Final_Data[4].Speed) > HIGH_SPEED || ABS(Final_Data[5].Speed) > HIGH_SPEED || ABS(Final_Data[6].Speed) > HIGH_SPEED)
    {
        count_flag = 1;
        count = 0;
    }

    else if(ABS(Final_Data[1].Speed) < LOW_SPEED && ABS(Final_Data[2].Speed) < LOW_SPEED && ABS(Final_Data[3].Speed) < LOW_SPEED
    && ABS(Final_Data[4].Speed) < LOW_SPEED && ABS(Final_Data[5].Speed) < LOW_SPEED && ABS(Final_Data[6].Speed) < LOW_SPEED && count == 0)
    {
        count_flag = 0;

        for (int i = 1; i < 4; ++i)
        {
            Memory_theta[i][Memory_count] = Final_Data[i].Angle - begin_pos[i];
        }

        times[Memory_count] = time;
        time = 0;

        Memory_count++;
        osDelay(300);
        count++;
    }
}
void by_Programming(void )
{
    speed_kd = 4.0f;

    for (int i = 1; i < 4; i++)
    {
        if(Memory_theta[1][count_t] == 0 && Memory_theta[2][count_t] == 0 && Memory_theta[3][count_t] == 0)
        {
            Pro_flag = 0;
            break;
        }
        TargetAngle[i] = Memory_theta[i][count_t];
    }

    count_t++;
    osDelay(2500);
}
float ABS(float a)
{
    if(a > 0) return a;
    else if(a < 0) return -a;
}