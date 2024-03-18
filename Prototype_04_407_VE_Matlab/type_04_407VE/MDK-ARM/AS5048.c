#include "AS5048.h"
#include "spi.h"


AS5048 AS5048s[AS5048_NUMBER];

void AS5048_init(int AS5048_ID,SPI_HandleTypeDef *spi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	
	AS5048 *AS5 = AS5048s + AS5048_ID;
	
	
	AS5->spi_number = spi;
	AS5->GPIOx = GPIOx;
	AS5->GPIO_Pin = GPIO_Pin;
	
	//°ó¶¨
}


uint16_t AS5048_Read(const int AS5048_ID, uint16_t registerAddress){

	uint8_t data[2] = {0, 0};
	uint8_t cmd[2] = {0, 0};
	
	cmd[1] = registerAddress & 0xFF;
	cmd[0] = ( registerAddress >> 8 ) & 0xFF;

	AS5048 *AS5 = AS5048s + AS5048_ID;
	//Send the command
	HAL_GPIO_WritePin(AS5->GPIOx ,AS5->GPIO_Pin, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(AS5->spi_number, cmd, 2, 1000);
	
	HAL_GPIO_WritePin(AS5->GPIOx ,AS5->GPIO_Pin, GPIO_PIN_SET);

		
	//Now read the response
	HAL_GPIO_WritePin(AS5->GPIOx ,AS5->GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(AS5->spi_number, cmd, data, 4, 1000);
	HAL_GPIO_WritePin(AS5->GPIOx ,AS5->GPIO_Pin, GPIO_PIN_SET);
	//printf("data[0] = %d data[1] = %d \n", data[0], data[1]);
	//Check if the error bit is set
	
	//Return the data, stripping the parity and error bits
	return ((( data[0] & 0xFF) << 8) | (data[1] & 0xFF)) & ~0xC000;
}

void AS5048_getREGValue(const int AS5048_ID){
	
	AS5048 *AS5 = AS5048s + AS5048_ID;
	AS5->angle =  AS5048_Read(AS5048_ID,SPI_REG_DATA);

}
