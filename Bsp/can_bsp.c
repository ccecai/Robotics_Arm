#include "can_bsp.h"
#include "fdcan.h"
#include "dm4310_drv.h"
#include "string.h"
#include "DeepMotor.h"

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN ʹ��
************************************************************************
**/
void can_bsp_init(void)
{
    can1_filter_init();
    can2_filter_init();
    can3_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //����FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN�˲�����ʼ��
************************************************************************
**/
void can1_filter_init(void)
{
    FDCAN_FilterTypeDef sFilterConfig1;
    sFilterConfig1.IdType = FDCAN_STANDARD_ID; //��׼ID
    sFilterConfig1.FilterIndex = 0;
    sFilterConfig1.FilterType = FDCAN_FILTER_RANGE; // �б�ģʽ  ����Ӧ��Ҫ�ĳ�����ģʽ
    sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig1.FilterID1 = 0X0000;
    sFilterConfig1.FilterID2 = 0X07FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1) != HAL_OK)
    {
        Error_Handler();
    }
    // ��������Ŀ���Ҫ�����ж�ʹ��ǰ��
    HAL_FDCAN_Start(&hfdcan1);
    /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

void can2_filter_init(void)
{
    FDCAN_FilterTypeDef sFilterConfig1;
    sFilterConfig1.IdType = FDCAN_STANDARD_ID; //��׼ID
    sFilterConfig1.FilterIndex = 1;
    sFilterConfig1.FilterType = FDCAN_FILTER_RANGE; // �б�ģʽ  ����Ӧ��Ҫ�ĳ�����ģʽ
    sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig1.FilterID1 = 0X0000;
    sFilterConfig1.FilterID2 = 0X07FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig1) != HAL_OK)
    {
        Error_Handler();
    }
    // ��������Ŀ���Ҫ�����ж�ʹ��ǰ��
    HAL_FDCAN_Start(&hfdcan2);
    /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

void can3_filter_init(void)
{
    FDCAN_FilterTypeDef sFilterConfig1;
    sFilterConfig1.IdType = FDCAN_STANDARD_ID; //��׼ID
    sFilterConfig1.FilterIndex = 2;
    sFilterConfig1.FilterType = FDCAN_FILTER_RANGE; // �б�ģʽ  ����Ӧ��Ҫ�ĳ�����ģʽ
    sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig1.FilterID1 = 0X0000;
    sFilterConfig1.FilterID2 = 0X07FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig1) != HAL_OK)
    {
        Error_Handler();
    }
    // ��������Ŀ���Ҫ�����ж�ʹ��ǰ��
    HAL_FDCAN_Start(&hfdcan3);
    /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan��FDCAN���
* @param:       id��CAN�豸ID
* @param:       data�����͵�����
* @param:       len�����͵����ݳ���
* @retval:     	void
* @details:    	��������
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	FDCAN_TxHeaderTypeDef TxHeader;
	
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;																// ��׼ID 
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;														// ����֡ 
  TxHeader.DataLength = len << 16;																		// �������ݳ��� 
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										// ���ô���״ָ̬ʾ 								
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;															// �������ɱ䲨���� 
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;															// ��ͨCAN��ʽ 
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										// ���ڷ����¼�FIFO����, ���洢 
  TxHeader.MessageMarker = 0x00; 			// ���ڸ��Ƶ�TX EVENT FIFO����ϢMaker��ʶ����Ϣ״̬����Χ0��0xFF                
    
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) 
		return 1;//����
	return 0;	
}

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;                 // CAN ID
    TxHeader.IdType =  FDCAN_STANDARD_ID ;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    if(len<=8)
    {
        TxHeader.DataLength = len<<16;     // ���ͳ��ȣ�8byte
    }
    else  if(len==12)
    {
        TxHeader.DataLength =FDCAN_DLC_BYTES_12;
    }
    else  if(len==16)
    {
        TxHeader.DataLength =FDCAN_DLC_BYTES_16;

    }
    else  if(len==20)
    {
        TxHeader.DataLength =FDCAN_DLC_BYTES_20;
    }
    else  if(len==24)
    {
        TxHeader.DataLength =FDCAN_DLC_BYTES_24;
    }else  if(len==48)
    {
        TxHeader.DataLength =FDCAN_DLC_BYTES_48;
    }else  if(len==64)
    {
        TxHeader.DataLength =FDCAN_DLC_BYTES_64;
    }

    TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//�������л��رգ�
    TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // CAN2.0
    TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;//��Ϣ���

    // ����CANָ��
    if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
    {
        // ����ʧ�ܴ���
        Error_Handler();
    }
    return 0;
}

/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan��FDCAN���
* @param:       buf���������ݻ���
* @retval:     	���յ����ݳ���
* @details:    	��������
************************************************************************
**/
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
  if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &fdcan_RxHeader, buf)!=HAL_OK)
		return 0;//��������
  return fdcan_RxHeader.DataLength>>16;	
}
/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan��FDCAN���
* @param:       RxFifo0ITs���жϱ�־λ
* @retval:     	void
* @details:    	HAL���FDCAN�жϻص�����
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
		if(hfdcan == &hfdcan1)
		{
            fdcan1_rx_callback();
		}
		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
		if(hfdcan == &hfdcan3)
		{
			fdcan3_rx_callback();
		}
	}
}
/**
************************************************************************
* @brief:      	fdcan_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	���û����õĽ���������
************************************************************************
**/
void fdcan1_rx_callback(void)
{
    HAL_StatusTypeDef HAL_RetVal;
    FDCAN_RxHeaderTypeDef RxHeader;
    union_64 rxdata;
    /*����ż�¼*/
    static uint8_t index;

    HAL_RetVal = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, rxdata.data_8);
    if(HAL_RetVal == HAL_OK)
    {

        if(RxHeader.Identifier >= Control_ID1_Receive && RxHeader.Identifier <= Control_ID6_Receive)
        {

            index = (RxHeader.Identifier - 0x10) & 0x01F;

            FeedBack_Data.Angle = rxdata.data_8[0] | rxdata.data_8[1] << 8 | ((rxdata.data_8[2] << 16) & 0x0FFFFF);
            FeedBack_Data.Speed = ((rxdata.data_8[2] >> 4) & 0x0F) | rxdata.data_8[3] << 4 | rxdata.data_8[4] << 12;
            FeedBack_Data.Torque = rxdata.data_8[5] | rxdata.data_8[6] << 8;
            FeedBack_Data.Temperature_flag = rxdata.data_8[7] & 0x01;
            FeedBack_Data.Temperature = (rxdata.data_8[7] >> 1) & 0x7F;

            ReceiveData_Process(&FeedBack_Data,index);
        }

    }
    __HAL_FDCAN_ENABLE_IT(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}
uint8_t rx_data2[8] = {0};
void fdcan2_rx_callback(void)
{

    /* Retrieve Rx messages from RX FIFO0 */
    memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
    HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader2, g_Can2RxData);
    switch(RxHeader2.Identifier)
    { //�������IDΪ0
        case 0:dm4310_fbdata(&motor, g_Can2RxData, RxHeader2.DataLength);break;
        default:break;

    }
}

uint8_t rx_data3[8] = {0};
void fdcan3_rx_callback(void)
{
	fdcanx_receive(&hfdcan3, rx_data3);
}
