//
// Created by 1 on 2024-05-04.
//
#include "Screen.h"
#include "uart_bsp.h"

uint8_t Screen_Data[10] = {0};
uint8_t Pro_flag = 0;

void Screen_DataProcess(void)
{
    if(Screen_Data[0] == 0xa0 && Screen_Data[1] == 0x11 && Screen_Data[2] == 0xaf)
    {
        Pro_flag = 1;
    }
    else if(Screen_Data[0] == 0xa0 && Screen_Data[1] == 0x12 && Screen_Data[2] == 0xaf)
    {
        output_mode = Gravity_compensation_MODE;
    }
    else if(Screen_Data[0] == 0xa0 && Screen_Data[1] == 0x13 && Screen_Data[2] == 0xaf)
    {
        IF_kinematics(prx,pry,prz);
        output_mode = IF_MODE;
    }
    else if(Screen_Data[0] == 0xa0 && Screen_Data[1] == 0x14 && Screen_Data[2] == 0xaf && Pro_flag == 1)
    {
        output_mode = Gravity_compensation_MODE;
        Demonstration();
    }
    else if(Screen_Data[0] == 0xa0 && Screen_Data[1] == 0x15 && Screen_Data[2] == 0xaf && Pro_flag == 1)
    {
        output_mode = Demonstration_MODE;
        by_Programming();
    }
}