#include "AS5048.h"
#include "spi.h"


AS5048 AS5048s[AS5048_NUMBER];

void AS5048_init(int AS5048_ID,SPI_HandleTypeDef *spi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	
	AS5048 *AS5 = AS5048s + AS5048_ID -1;
	
	AS5->spi_number = spi;
	AS5->GPIOx = GPIOx;
	AS5->GPIO_Pin = GPIO_Pin;
	AS5->angle = 0;
	AS5->total_angle = 0;
	AS5->cirle = 0;
	AS5->last_angle = AS5->angle;
	AS5->delta_dis = 0;
	
}


uint16_t AS5048_Read(const int AS5048_ID, uint16_t registerAddress){

	uint8_t data[4] = {0, 0, 0,0};
	uint8_t cmd[4] = {0, 0,0,0};
	
	cmd[3] = registerAddress & 0xFF;
	cmd[2] = ( registerAddress >> 8 ) & 0xFF;
	cmd[1] = registerAddress & 0xFF;
	cmd[0] = ( registerAddress >> 8 ) & 0xFF;

	AS5048 *AS5 = AS5048s + AS5048_ID -1;

	//Now read the response
	HAL_GPIO_WritePin(AS5->GPIOx ,AS5->GPIO_Pin, GPIO_PIN_RESET);
	//HAL_Delay(1);
	HAL_SPI_TransmitReceive(AS5->spi_number, cmd, data, 4, 1000);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(AS5->GPIOx ,AS5->GPIO_Pin, GPIO_PIN_SET);

	return ((( data[1] & 0xFF) << 8) | (data[0] & 0xFF)) & ~0xC000;
}

void AS5048_getREGValue(const int AS5048_ID){
	
	AS5048 *AS5 = AS5048s + AS5048_ID -1;
	
	AS5->angle =  AS5048_Read(AS5048_ID,SPI_REG_DATA);

}


void AS5048_dataUpdate(const int AS5048_ID){

	AS5048 *AS5 = AS5048s + AS5048_ID -1;
	
	int diff = AS5->angle - AS5->last_angle; // cc distanc delta
	if (diff > 16000) {
		AS5->cirle--;
		AS5->total_angle = AS5->angle + AS5->cirle * 16384;
		AS5->delta_dis = diff - 16384;

	} else if (diff < -16000) {
		AS5->cirle++;
		AS5->total_angle = AS5->angle + AS5->cirle * 16384;
		AS5->delta_dis = diff + 16384;
		
	}else if (diff >=0) {
		AS5->total_angle += diff;
		AS5->delta_dis = diff;
	
	} else if (diff < 0) {
		AS5->total_angle += diff;
		AS5->delta_dis = diff;
	}	
	AS5->last_angle = AS5->angle;


}

