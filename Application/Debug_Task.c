//
// Created by 1 on 2024-06-06.
//
#include "Debug_Task.h"

uint8_t r = 1;
uint8_t g = 1;
uint8_t b = 1;

void Debug_Task(void )
{
    WS2812_Ctrl(r, g, b);
    r++;
    g += 5;
    b += 10;
    osDelay(1);
    r++;g++;b++;

    osDelay(100);
}

void Myinit(void )
{
    Power_OUT1_ON;
    Power_OUT2_ON;//ʹ�ܰ忨�ϵĵ�Դ
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2);
    can_bsp_init();
    HAL_TIM_Base_Start_IT(&htim2);

    joint_motor_init(&motor,1,MIT_MODE);

    osDelay(3000); //�ȴ���������������

    for(int i=0;i<6;i++)
    {
        enable_motor_mode(&hfdcan2, motor.para.id, MIT_MODE);//ʹ�ܵ��
        osDelay(20);
    }

    DH_init();

    AllMotor_ENABLE();
    Six_PID_Init();
    ChangeGainOfPID(15.0f,2.0f,0.0f,0.0f);
    for (int i = 1; i < 4; ++i) {
        PID_Set_KP_KI_KD(&Torque[i],0.64f,0,0.0f);
    }

}