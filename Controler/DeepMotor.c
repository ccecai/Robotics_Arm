//
// Created by 1 on 2024-05-02.
//
#include "DeepMotor.h"

ControlData FeedBack_Data;
FinalData Final_Data[7];
float TargetAngle[7] = {0};

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
    TargetAngle = 819.1875f * TargetAngle - 32767.5f;
    TargetSpeed = 204.7875f * TargetSpeed - 8191.5f;
    Kd = Kd * 5.0f;
    TargetTorque = 819.1875f * TargetTorque - 32767.5f;

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
    Final_Data[id].Angle = (80 / 1048575) * Data->Angle -40;
    Final_Data[id].Speed = (80 / 1048575) * Data->Speed -40;
    Final_Data[id].Torque = (80 / 65535) * Data->Torque - 40;
    Final_Data[id].Temperature_flag = Data->Temperature_flag;
    Final_Data[id].Temperature = (220 / 127) * Data->Temperature - 20;
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