//
// Created by 1 on 2024-06-06.
//

#ifndef ROBOTICS_ARM_DEBUG_TASK_H
#define ROBOTICS_ARM_DEBUG_TASK_H

#include "cmsis_os.h"
#include "ws2812.h"
#include "can_bsp.h"
#include "uart_bsp.h"
#include "pid.h"
#include "controller.h"
#include "DeepMotor.h"
#include "dm4310_drv.h"
#include "tim.h"
#include "gpio.h"

void Debug_Task(void );
void Myinit(void );

#endif //ROBOTICS_ARM_DEBUG_TASK_H
