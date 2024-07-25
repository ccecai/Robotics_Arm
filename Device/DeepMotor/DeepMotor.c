//
// Created by 1 on 2024-05-02.
//
#include "DeepMotor.h"

ControlData FeedBack_Data;
FinalData Final_Data[7];
float begin_pos[7] = {0};
float TargetAngle[7] = {0};
uint8_t output_mode = 1;

void CAN_CMD_MOTOR_DISABLE(FDCAN_HandleTypeDef *_hfdcan,uint32_t stdid)
{
    FDCAN_TxHeaderTypeDef  TxHeader;
    uint8_t TxData[8];

    TxHeader.Identifier=stdid;                       //32λID
    TxHeader.IdType=FDCAN_STANDARD_ID;                  //��׼ID
    TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //����֡
    TxHeader.DataLength= FDCAN_DLC_BYTES_0;             //���ݳ���8�ֽ�
    TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
    TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
    TxHeader.MessageMarker=0x00;                        //������

    // �ȴ�FDcan�Ŀ����� ���������жϷ��ͣ����ֻ��������ж����ڳ������б����ݷ��Ϳ���������ӵ�£�����һ�ν���if�жϾͳ������ˡ�
    while(HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) == 0);
    //���ͳɹ�����ʧ�ܾͿ�ס����
    if ( HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &TxHeader, TxData)!= HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_CMD_MOTOR_ENABLE(FDCAN_HandleTypeDef *_hfdcan,uint32_t stdid)
{
    FDCAN_TxHeaderTypeDef  TxHeader;
    uint8_t TxData[8];

    TxHeader.Identifier=stdid;                       //32λID
    TxHeader.IdType=FDCAN_STANDARD_ID;                  //��׼ID
    TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //����֡
    TxHeader.DataLength= FDCAN_DLC_BYTES_0;             //���ݳ���8�ֽ�
    TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
    TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
    TxHeader.MessageMarker=0x00;                        //������

    // �ȴ�FDcan�Ŀ����� ���������жϷ��ͣ����ֻ��������ж����ڳ������б����ݷ��Ϳ���������ӵ�£�����һ�ν���if�жϾͳ������ˡ�
    while(HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) == 0);
    //���ͳɹ�����ʧ�ܾͿ�ס����
    if ( HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &TxHeader, TxData)!= HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_CMD_MOTOR_CONTROL(FDCAN_HandleTypeDef *_hfdcan,float TargetAngle,float TargetSpeed,
                           float Kp,float Kd,float TargetTorque,float stdid)
{
    TargetAngle = 65535 * TargetAngle / 80.0f + 65535.0f / 2;
    TargetSpeed = 16383 * TargetSpeed / 80.0f + 16383.0f / 2;
    Kd = Kd * 5.0f;
    TargetTorque = 65535 * TargetTorque / 80.0f + 65535.0f / 2;

    FDCAN_TxHeaderTypeDef  TxHeader;
    uint8_t TxData[8];

    TxHeader.Identifier=stdid;                       //32λID
    TxHeader.IdType=FDCAN_STANDARD_ID;                  //��׼ID
    TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //����֡
    TxHeader.DataLength= FDCAN_DLC_BYTES_8;             //���ݳ���8�ֽ�
    TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
    TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
    TxHeader.MessageMarker=0x00;                        //������

    TxData[0] = (int )TargetAngle;
    TxData[1] = (int )TargetAngle >> 8;

    TxData[2] = (int )TargetSpeed;
    TxData[3] = (((int )TargetSpeed >> 8 ) & 0x3F) | ((int )Kp << 6);

    TxData[4] = (int )Kp >> 2;
    TxData[5] = (int )Kd;

    TxData[6] = (int )TargetTorque;
    TxData[7] = (int )TargetTorque >> 8;


    // �ȴ�FDcan�Ŀ����� ���������жϷ��ͣ����ֻ��������ж����ڳ������б����ݷ��Ϳ���������ӵ�£�����һ�ν���if�жϾͳ������ˡ�
    while(HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) == 0);
    //���ͳɹ�����ʧ�ܾͿ�ס����
    if ( HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &TxHeader, TxData)!= HAL_OK)
    {
        Error_Handler();
    }

}

void ReceiveData_Process(ControlData *Data,uint8_t id)
{
    Final_Data[id].Angle = 80.0f * Data->Angle / 1048575 -40;
    Final_Data[id].Speed = 80.0f * Data->Speed / 1048575 -40;
    Final_Data[id].Torque = 80.0f * Data->Torque / 65535 -40;
    Final_Data[id].Temperature_flag = Data->Temperature_flag;
    Final_Data[id].Temperature = (uint8_t )(220 * Data->Temperature / 127 - 20);

}

void AllMotor_ENABLE(void)
{
    CAN_CMD_MOTOR_ENABLE(&hfdcan1,Able_ID1);
    osDelay(2);
    CAN_CMD_MOTOR_ENABLE(&hfdcan1,Able_ID2);
    osDelay(2);
    CAN_CMD_MOTOR_ENABLE(&hfdcan1,Able_ID3);
    osDelay(2);
    CAN_CMD_MOTOR_ENABLE(&hfdcan1,Able_ID4);
    osDelay(2);
    CAN_CMD_MOTOR_ENABLE(&hfdcan1,Able_ID5);
    osDelay(2);
    CAN_CMD_MOTOR_ENABLE(&hfdcan1,Able_ID6);
    osDelay(2);

    CAN_CMD_MOTOR_CONTROL(&hfdcan1,0,0.0f,0.0f,0.0f,0.0f,Control_ID1);
    osDelay(5);
    CAN_CMD_MOTOR_CONTROL(&hfdcan1,0,0.0f,0.0f,0.0f,0.0f,Control_ID2);
    osDelay(5);
    CAN_CMD_MOTOR_CONTROL(&hfdcan1,0,0.0f,0.0f,0.0f,0.0f,Control_ID3);
    osDelay(5);

    for (int i = 1; i < 7; ++i)
    {
        begin_pos[i] = Final_Data[i].Angle;
    }
}

void AllMotor_DISABLE(void)
{
    CAN_CMD_MOTOR_DISABLE(&hfdcan1,Disable_ID1);
    osDelay(2);
    CAN_CMD_MOTOR_DISABLE(&hfdcan1,Disable_ID2);
    osDelay(2);
    CAN_CMD_MOTOR_DISABLE(&hfdcan1,Disable_ID3);
    osDelay(2);
    CAN_CMD_MOTOR_DISABLE(&hfdcan1,Disable_ID4);
    osDelay(2);
    CAN_CMD_MOTOR_DISABLE(&hfdcan1,Disable_ID5);
    osDelay(2);
    CAN_CMD_MOTOR_DISABLE(&hfdcan1,Disable_ID6);
    osDelay(2);

}