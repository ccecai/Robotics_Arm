//
// Created by 1 on 2024-06-06.
//
#include "Motor_Task.h"

void Motor_Task(void)
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

            AngleLoop[1].Output_limit = 1.5f;
            AngleLoop[2].Output_limit = 1.5f;
            AngleLoop[3].Output_limit = 1.5f;

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

            AngleLoop[1].Output_limit = 1.5f;
            AngleLoop[2].Output_limit = 1.5f;
            AngleLoop[3].Output_limit = 1.5f;

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

        case NO_MODE:
//            SetPoint_IMU(&Torque[1],0);
//            PID_PosLocCalc(&Torque[1],Final_Data[1].Angle,1);
//            SetPoint_IMU(&Torque[2],0);
//            PID_PosLocCalc(&Torque[2],Final_Data[2].Angle,2);
//            SetPoint_IMU(&Torque[3],0);
//            PID_PosLocCalc(&Torque[3],Final_Data[3].Angle,3);
//
//            Torque[1].Output_limit = 2.0f;
//            Torque[2].Output_limit = 2.0f;
//            Torque[3].Output_limit = 2.0f;

            CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,0.0f,0.0f,0.0f,0,Control_ID1);
            osDelay(2);
            CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,0.0f,0.0f,0.0f,0,Control_ID2);
            osDelay(2);
            CAN_CMD_MOTOR_CONTROL(&hfdcan1,0.0f,0.0f,0.0f,0.0f,0,Control_ID3);
            osDelay(2);

            break;

        default:
            break;
    }

    osDelay(2);
}