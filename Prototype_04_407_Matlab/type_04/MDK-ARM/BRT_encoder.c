#include "BRT_encoder.h"
#include "main.h"
#include "math.h"
extern CAN_HandleTypeDef    hcan1;  //SET THE CAN YOU USE HERE
//extern CAN_HandleTypeDef    hcan2; 
encoder_data_t encoder_data[4];
static uint8_t         encoder_can_send_data[4];

static CAN_TxHeaderTypeDef  encoder_tx_message;




		
/// @brief actually not use 
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st1;
	
	
		uint32_t StdId1 =0x001;						
		uint32_t StdId2 =0x003;
		uint32_t StdId3 =0x002;
		uint32_t StdId4 =0x004;
	
	
    can_filter_st1.FilterActivation = ENABLE;
    can_filter_st1.FilterMode =  CAN_FILTERMODE_IDLIST;
    can_filter_st1.FilterScale = CAN_FILTERSCALE_16BIT;
//    can_filter_st1.FilterIdHigh = 0x0000;
//    can_filter_st1.FilterIdLow = 0x0000;
//    can_filter_st1.FilterMaskIdHigh = 0x0000;
//    can_filter_st1.FilterMaskIdLow = 0x0000;
//	
		 can_filter_st1.FilterIdHigh = StdId1<<5;	 //4个标准CAN ID分别放入到4个存储中
		 can_filter_st1.FilterIdLow = StdId2<<5;
		 can_filter_st1.FilterMaskIdHigh = StdId3<<5;
		 can_filter_st1.FilterMaskIdLow = StdId4<<5;

	
	
    can_filter_st1.FilterBank = 0;
    can_filter_st1.FilterFIFOAssignment = CAN_RX_FIFO0;
	//	can_filter_st1.SlaveStartFilterBank = 14;
  	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	
	
//		CAN_FilterTypeDef can_filter_st2;
//    can_filter_st2.FilterActivation = ENABLE;
//    can_filter_st2.FilterMode = CAN_FILTERMODE_IDMASK;
//    can_filter_st2.FilterScale = CAN_FILTERSCALE_32BIT;
//    can_filter_st2.FilterIdHigh = 0x0000;
//    can_filter_st2.FilterIdLow = 0x0000;
//    can_filter_st2.FilterMaskIdHigh = 0x0000;
//    can_filter_st2.FilterMaskIdLow = 0x0000;
//    can_filter_st2.FilterBank = 14;
//    can_filter_st2.FilterFIFOAssignment = CAN_RX_FIFO0;
//		can_filter_st2.SlaveStartFilterBank = 28;
//  	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st2);
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}


///// @brief hal库CAN回调函数,接收ENCODER数据
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	
    CAN_RxHeaderTypeDef rx_header;
		if (hcan->Instance == hcan1.Instance){
  
		uint8_t rx_data[7];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
		static uint8_t ec_n = 0;
    ec_n = rx_header.StdId - 1;
			
			
    get_encoder_measure(encoder_data+ec_n, rx_data);
	
	
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
			
		

		}
		
//		if (hcan->Instance == hcan2.Instance){
//  
//		uint8_t rx_data[7];

//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//		
//		static uint8_t ec_n = 0;
//    ec_n = rx_header.StdId - 0;
//		get_encoder_measure(encoder_data, rx_data);
//	
//	
//		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

//		}

//		
		
		}

/// @brief ECD数据的读取
void	 get_encoder_measure(encoder_data_t *ptr,uint8_t *data)                                    
    {                                                                   
					
					ptr->last_ecd =  ptr->ecd;
				  ptr->last_tt_ecd = ptr->tt_ecd;
					ptr->ecd = (int)((data)[6] << 24 | (data)[5] << 16 | (data)[4] << 8 |(data)[3] ); 
					if(ptr->last_ecd-ptr->ecd >20000){
					
					ptr->circle += 1;
						
					}else if(ptr->last_ecd-ptr->ecd < -20000){
					
					ptr->circle -= 1;
						
					} 
					ptr->tt_ecd = ptr->ecd + ptr->circle * 24576;
					
					//ecd_gb = (int)((data)[6] << 24 | (data)[5] << 16 | (data)[4] << 8 |(data)[3] );
    }

/// @brief hal库CAN回调函数,发送ENCODER数据
void CAN_CMD_ENCODER(uint8_t ID,uint32_t STD)
{
		
    uint32_t send_mail_box;
    encoder_tx_message.StdId = STD;
    encoder_tx_message.IDE = CAN_ID_STD;
    encoder_tx_message.RTR = CAN_RTR_DATA;
    encoder_tx_message.DLC = 0x04;
    encoder_can_send_data[0] = 0x04;
    encoder_can_send_data[1] = ID;
		encoder_can_send_data[2] = 0x01;
		encoder_can_send_data[3] = 0x00;
	
    HAL_CAN_AddTxMessage(&hcan1,&encoder_tx_message, encoder_can_send_data, &send_mail_box);
}


void CAN_CMD_ENCODER2()
{
		
//    uint32_t send_mail_box;
//    encoder_tx_message.StdId = 0x01;
//    encoder_tx_message.IDE = CAN_ID_STD;
//    encoder_tx_message.RTR = CAN_RTR_DATA;
//    encoder_tx_message.DLC = 0x04;
//    encoder_can_send_data[0] = 0x04;
//    encoder_can_send_data[1] = 0x01;
//		encoder_can_send_data[2] = 0x01;
//		encoder_can_send_data[3] = 0x00;
//	
//    HAL_CAN_AddTxMessage(&hcan2,&encoder_tx_message, encoder_can_send_data, &send_mail_box);
}
