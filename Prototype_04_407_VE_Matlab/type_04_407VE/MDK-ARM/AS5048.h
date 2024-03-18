#ifndef __AS5048_H
#define __AS5048_H

#include "main.h"

#define AS5048_NUMBER 2  



#define SPI_CMD_READ				0x4000 // flag indicating read attempt
#define SPI_CMD_WRITE				0x8000 // flag indicating write attempt
#define SPI_NOP							0x0000//B0000000000000000 No operation dummy information
#define SPI_REG_AGC					0x7ffd // agc register when using SPI 
#define SPI_REG_MAG					0x7ffe // magnitude register when using SPI
#define SPI_REG_DATA				0xffff // data register when using SPI
#define SPI_REG_CLRERR			0x4001 // clear error register when using SPI
#define SPI_REG_ZEROPOS_HI	0x0016 // zero position register high byte
#define SPI_REG_ ZEROPOS_LO	0x0017 // zero position register low byte



void AS5048_init(int AS5048_ID,SPI_HandleTypeDef *spi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
uint16_t AS5048_Read(const int AS5048_ID, uint16_t registerAddress);
void AS5048_getREGValue(const int AS5048_ID);

/**
 * @brief AS5048_STRUCT
 */
typedef struct {
  
	
	const int AS5048_ID;
  SPI_HandleTypeDef *spi_number;       ///< SPI
	GPIO_TypeDef *GPIOx; ///<GPIO_CS
	uint16_t GPIO_Pin; ///<GPIO_PIN_CS
  int    angle;
} AS5048;

/**
 * @brief AS5048_OBJECT_
 */

extern AS5048 AS5048s[AS5048_NUMBER];

#endif