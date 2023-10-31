#ifndef BSP_CAN_H
#define BSP_CAN_H
#include <stdint.h>
#include "can.h"

typedef struct
{
    int ecd;            
    int last_ecd;
		int tt_ecd;
		int circle ;
		int last_tt_ecd;
} encoder_data_t;


void CAN_CMD_ENCODER(uint8_t ID,uint32_t STD);


void can_filter_init(void);

void get_encoder_measure(encoder_data_t *ptr,uint8_t *data ) ;

extern encoder_data_t encoder_data[4];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void CAN_CMD_ENCODER2(void);
#endif
