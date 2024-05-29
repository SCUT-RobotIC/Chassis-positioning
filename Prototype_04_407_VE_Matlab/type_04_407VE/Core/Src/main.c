/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "YIS130.h"
#include "arm_math.h"
#include "IM_TEST.h"
#include "stdio.h"
#include "AS5048.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USART_CR3_RXFTIE ((uint32_t)0x00008000)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t receive_buff[8] = {0}; // init  
int receivefactor[2];
// header x_low x_high y y yaw yaw footer
uint8_t tx_buff[255];


extern MPU_DATA mpu_data[4];
extern AS5048 AS5048s[AS5048_NUMBER];
float i = 0;
extern float ACCX,ACCY,ACCZ;
//float tt_x = 0;
//float tt_y = 0;
//float tt_x_real = 0;
//float tt_y_real = 0;


int add = 0;
int times = 0;

int fputc(int ch, FILE *f){
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

typedef struct struct_message
{
  uint8_t header;
  uint8_t parity;
  uint8_t data[6];
  uint8_t footer;
} DataPacket;

DataPacket DataRe;
uint8_t USART_FLAG = 0;


uint8_t USART1_RX_BUF[100]; 
uint16_t USART1_RX_STA = 0; 
uint8_t aRxBuffer1[1];		  
UART_HandleTypeDef UART1_Handler; 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rcv_buf[8]={0};
int rcv_err = 3;
uint8_t *prcv = rcv_buf;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 
	HAL_UART_Receive_DMA(&huart1,rcv_buf,8);	
	
	AS5048_init(1,&hspi1,GPIOA,GPIO_PIN_4);
	AS5048_init(2,&hspi2,GPIOB,GPIO_PIN_12);
	
	mpu_data[0].cali = 1; // �Ȳ���
	mpu_data[0].vel[0] = 0;
	mpu_data[0].vel[1] = 0;
   mpu_data[0].REAL_YAW_SET = 0;
	 mpu_data[0].REAL_YAW_MARK = 0;
		
	can_filter_init();
	IM_TEST_initialize();
	
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim11);

		
//	SelfCalibration();
	
//	HAL_Delay(100);
	

//  mpu_data[0].PITCH_ANGLE_BEG = mpu_data[0].PITCH_ANGLE;
//  mpu_data[0].YAW_ANGLE_BEG =   mpu_data[0].YAW_ANGLE;
//  mpu_data[0].ROLL_ANGLE_BEG =  mpu_data[0].ROLL_ANGLE;
//  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	
	while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//			while  (huart->Instance == USART1)
//		{
//				USART1_RX_BUF[USART1_RX_STA] = aRxBuffer1[0];
//				if (USART1_RX_STA == 0 && USART1_RX_BUF[USART1_RX_STA] != 0x0F) 	
//				{			
//					HAL_UART_Receive_DMA(&huart1,aRxBuffer1,1);	
//					break; //
//				}
//				USART1_RX_STA++;
//			HAL_UART_Receive_DMA(&huart1,aRxBuffer1,1);
//			if (USART1_RX_STA > 100) USART1_RX_STA = 0;  //
//			if (USART1_RX_BUF[0] == 0x0F && USART1_RX_BUF[7] == 0xAA && USART1_RX_STA == 8)	//检测包头包尾以及数据包长度
//			{
//				DATARELOAD(USART1_RX_BUF);
//				receivefactor[1]=1;
//				USART1_RX_STA = 0;
//			}
//			else if(!(USART1_RX_BUF[0] == 0x0F && USART1_RX_BUF[7] == 0xAA) && USART1_RX_STA == 8){
//				for(int i=0;i<8;i++)
//					USART1_RX_BUF[i] = 0;
//				USART1_RX_STA = 0;
//			}
//			break;
//		}
//}

//因为只需要接收8字节数据，所以lw把接收函数改了一下
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		if(0x0F==rcv_buf[0]&&0xAA==rcv_buf[7]){
			DATARELOAD(rcv_buf);
			if(rcv_err>0) rcv_err --;
		}
		for(int i=0;i<8;i++){
			rcv_buf[i]=0;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim14)){
			//if(rcv_err>0){
				ClearUARTErrors(USART1);//清除串口错误标志
			//}
			
    }
		
		if (htim == (&htim13)){
//			arm_fir_f32_lp();
			
		 }
		 
		 if (htim == (&htim11)){
			
			 if(mpu_data[0].cali == 1){
//					rtU.X_ACCIN  = mpu_data[0].acc_cali[0];
//					rtU.Y_ACCIN  = mpu_data[0].acc_cali[1];
				 
					if(times >= 500){
						
						add++;
						AS5048_getREGValue(1);
					//	HAL_Delay(1);
						AS5048_dataUpdate(1);	
				//		HAL_Delay(1);
						AS5048_getREGValue(2);
				//		HAL_Delay(1);
						AS5048_dataUpdate(2);	
				//		HAL_Delay(1);
						
			
						float temp_zero = mpu_data[0].REAL_YAW_SET - 180 - (180 - mpu_data[0].REAL_YAW_MARK);
						if(temp_zero < 0){ temp_zero += 360; };
//						float temp_180  = temp_zero - 180 ; 
//						if(temp_180 < 0){ temp_180 += 360; };

            if(mpu_data[0].YAW_ANGLE >= 0 && mpu_data[0].YAW_ANGLE < temp_zero){
              mpu_data[0].REAL_YAW = 360 - (temp_zero - mpu_data[0].YAW_ANGLE);
            }
            else if(mpu_data[0].YAW_ANGLE >= temp_zero ){
              mpu_data[0].REAL_YAW = mpu_data[0].YAW_ANGLE - temp_zero;
            }
           
						if(mpu_data[0].REAL_YAW > 180 ){
							
							mpu_data[0].REAL_YAW =  mpu_data[0].REAL_YAW - 360;
						}
						
						
            rtU.W1 = -AS5048s[1].delta_dis;
            rtU.W2 = AS5048s[0].delta_dis;
            rtU.DEG = mpu_data[0].REAL_YAW;
						
            mpu_data[0].Y_tt += rtY.YOUT ;//*0.014373;
            mpu_data[0].X_tt += rtY.XOUT ;//*0.014373;
						mpu_data[0].REAL_Y =  mpu_data[0].Y_tt * 0.014373;
						mpu_data[0].REAL_X  =mpu_data[0].X_tt * 0.014373;
						// x y yaw
						
						if(add >= 50){
							printf("%f %f %f\r\n",mpu_data[0].REAL_X,mpu_data[0].REAL_Y,mpu_data[0].REAL_YAW);
						  add = 0;
						}
						
					}else{
						
							AS5048_getREGValue(1);
						//	HAL_Delay(1);
							AS5048_dataUpdate(1);	
					//		HAL_Delay(1);
							AS5048_getREGValue(2);
					//		HAL_Delay(1);
							AS5048_dataUpdate(2);
					  mpu_data[0].vel[0] = 0;
				    mpu_data[0].vel[1] = 0;
						times ++ ;
					}

				IM_TEST_step();
			 			 
		 }
		}
}

void ClearUARTErrors(USART_TypeDef *USARTx) {
    // 清除奇偶校验错误
    if (USARTx->SR & USART_SR_PE) {
        (void)USARTx->DR;
    }
    // 清除帧错误
    if (USARTx->SR & USART_SR_FE) {
        (void)USARTx->DR;
    }
    // 清除 noise error
    if (USARTx->SR & USART_SR_NE) {
        (void)USARTx->DR;
    }
    // 清除 overrun error
    if (USARTx->SR & USART_SR_ORE) {
        (void)USARTx->DR;
    }
    // 重新使能串口
    USARTx->CR1 |= USART_CR1_UE;
    // 重新使能错误中断
    USARTx->CR3 |= USART_CR3_EIE;
    // 重新使能接收超过FIFO阈值中断
    USARTx->CR3 |= USART_CR3_RXFTIE;
		// 重新打开串口接收
		HAL_UART_Receive_DMA(&huart1,rcv_buf,8);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
