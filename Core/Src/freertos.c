/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_bsp.h"
#include "can_bsp.h"
#include "BMI088driver.h"
#include "adc_modlue.h"
#include "usart.h"
#include "ws2812.h"
#include "controller.h"
#include "DeepMotor.h"
#include "pid.h"
#include "Screen.h"
#include "Programming_by_Demonstration.h"
#include "printf.h"
#include "dm4310_drv.h"
#include "gpio.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t r = 1;
uint8_t g = 1;
uint8_t b = 1;
/* USER CODE END Variables */
osThreadId StartDebugTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId MotorOutputTaskHandle;
uint32_t myTask04Buffer[ 512 ];
osStaticThreadDef_t myTask04ControlBlock;
osThreadId ArmTaskHandle;
uint32_t myTask05Buffer[ 256 ];
osStaticThreadDef_t myTask05ControlBlock;
osThreadId RCTaskHandle;
uint32_t myTask06Buffer[ 1024 ];
osStaticThreadDef_t myTask06ControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebug(void const * argument);
void MotorOutput(void const * argument);
void Arm(void const * argument);
void RC(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StartDebugTask */
  osThreadStaticDef(StartDebugTask, StartDebug, osPriorityLow, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  StartDebugTaskHandle = osThreadCreate(osThread(StartDebugTask), NULL);

  /* definition and creation of MotorOutputTask */
  osThreadStaticDef(MotorOutputTask, MotorOutput, osPriorityHigh, 0, 512, myTask04Buffer, &myTask04ControlBlock);
  MotorOutputTaskHandle = osThreadCreate(osThread(MotorOutputTask), NULL);

  /* definition and creation of ArmTask */
  osThreadStaticDef(ArmTask, Arm, osPriorityRealtime, 0, 256, myTask05Buffer, &myTask05ControlBlock);
  ArmTaskHandle = osThreadCreate(osThread(ArmTask), NULL);

  /* definition and creation of RCTask */
  osThreadStaticDef(RCTask, RC, osPriorityBelowNormal, 0, 1024, myTask06Buffer, &myTask06ControlBlock);
  RCTaskHandle = osThreadCreate(osThread(RCTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

    vTaskSuspend(MotorOutputTaskHandle);
    vTaskSuspend(ArmTaskHandle);
    vTaskSuspend(RCTaskHandle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDebug */
/**
  * @brief  Function implementing the StartDebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDebug */
void StartDebug(void const * argument)
{
  /* USER CODE BEGIN StartDebug */
    Power_OUT1_ON;
    Power_OUT2_ON;//使能板卡上的
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2);
    can_bsp_init();
    HAL_TIM_Base_Start_IT(&htim2);

    joint_motor_init(&motor,1,MIT_MODE);

    osDelay(1000);

    for(int i=0;i<6;i++)
    {
        enable_motor_mode(&hfdcan2, motor.para.id, MIT_MODE);//使能电机
        osDelay(20);
    }

    DH_init();

    AllMotor_ENABLE();
    Six_PID_Init();
    ChangeGainOfPID(15.0f,2.0f,0.0f,0.0f);
    for (int i = 1; i < 4; ++i) {
        PID_Set_KP_KI_KD(&Torque[i],0.64f,0,0.0f);
    }


    vTaskResume(RCTaskHandle);
    vTaskResume(ArmTaskHandle);
    vTaskResume(MotorOutputTaskHandle);
  /* Infinite loop */
  for(;;)
  {
      WS2812_Ctrl(r, g, b);
      r++;
      g += 5;
      b += 10;
      osDelay(1);
      r++;g++;b++;

    osDelay(100);
  }
  /* USER CODE END StartDebug */
}

/* USER CODE BEGIN Header_MotorOutput */
/**
* @brief Function implementing the MotorOutputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorOutput */
void MotorOutput(void const * argument)
{
  /* USER CODE BEGIN MotorOutput */
  /* Infinite loop */
  for(;;)
  {
      switch (output_mode)
      {
          case IF_MODE:

              SetPoint(&AngleLoop[1],theta1,1);
              PID_PosLocCalc(&AngleLoop[1],Final_Data[1].Angle,1);
              SetPoint(&AngleLoop[2],theta2,2);
              PID_PosLocCalc(&AngleLoop[2],Final_Data[2].Angle,2);
              SetPoint(&AngleLoop[3],theta3,3);
              PID_PosLocCalc(&AngleLoop[3],Final_Data[3].Angle,3);

              AngleLoop[1].Output_limit = 2.5f;
              AngleLoop[2].Output_limit = 2.5f;
              AngleLoop[3].Output_limit = 2.5f;

              CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,AngleLoop[1].Out_put,0.0f,4.2f,0.0f,Control_ID1);
              osDelay(2);
              CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,AngleLoop[2].Out_put,0.0f,4.2f,0.0f,Control_ID2);
              osDelay(2);
              CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,AngleLoop[3].Out_put,0.0f,4.2f,0.0f,Control_ID3);
              osDelay(2);

              break;

          case Demonstration_MODE:

              SetPoint(&AngleLoop[1],TargetAngle[1],1);
              PID_PosLocCalc(&AngleLoop[1],Final_Data[1].Angle,1);
              SetPoint(&AngleLoop[2],TargetAngle[2],2);
              PID_PosLocCalc(&AngleLoop[2],Final_Data[2].Angle,2);
              SetPoint(&AngleLoop[3],TargetAngle[3],3);
              PID_PosLocCalc(&AngleLoop[3],Final_Data[3].Angle,3);

              AngleLoop[1].Output_limit = 2.0f;
              AngleLoop[2].Output_limit = 2.0f;
              AngleLoop[3].Output_limit = 2.0f;

              CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,AngleLoop[1].Out_put,0.0f,speed_kd,0.0f,Control_ID1);
              osDelay(2);
              CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,AngleLoop[2].Out_put,0.0f,speed_kd,0.0f,Control_ID2);
              osDelay(2);
              CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,AngleLoop[3].Out_put,0.0f,speed_kd,0.0f,Control_ID3);
              osDelay(2);

              break;

          case Gravity_compensation_MODE:

              SetPoint_IMU(&Torque[1],0);
              PID_PosLocCalc(&Torque[1],Final_Data[1].Angle,1);
              SetPoint_IMU(&Torque[2],0);
              PID_PosLocCalc(&Torque[2],Final_Data[2].Angle,2);
              SetPoint_IMU(&Torque[3],0);
              PID_PosLocCalc(&Torque[3],Final_Data[3].Angle,3);

              Torque[1].Output_limit = 2.0f;
              Torque[2].Output_limit = 2.0f;
              Torque[3].Output_limit = 2.0f;

              CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,0.0f,0.0f,0.0f,Torque[1].Out_put,Control_ID1);
              osDelay(2);
              CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,0.0f,0.0f,0.0f,Torque[2].Out_put,Control_ID2);
              osDelay(2);
              CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,0.0f,0.0f,0.0f,Torque[3].Out_put,Control_ID3);
              osDelay(2);

              break;

          default:
              break;
      }

      osDelay(2);
  }
  /* USER CODE END MotorOutput */
}

/* USER CODE BEGIN Header_Arm */
/**
* @brief Function implementing the ArmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Arm */
void Arm(void const * argument)
{
  /* USER CODE BEGIN Arm */
  /* Infinite loop */
  for(;;)
  {
      Screen_DataProcess();

    osDelay(5);
  }
  /* USER CODE END Arm */
}

/* USER CODE BEGIN Header_RC */
/**
* @brief Function implementing the RCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC */
void RC(void const * argument)
{
  /* USER CODE BEGIN RC */
  /* Infinite loop */
  for(;;)
  {
      RC_process();

//      usart_printf("%f,%f,%f,%f,%f,%f,%f,%f\n",times[0] / 1000,times[1] / 1000,times[2] / 1000,times[3] / 1000,Memory_theta[1][0],Memory_theta[1][1],Memory_theta[1][2],Memory_theta[1][3]);

    osDelay(1);
  }
  /* USER CODE END RC */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
