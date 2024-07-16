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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern MPU_DATA mpu_data[4];
extern AS5048 AS5048s[AS5048_NUMBER];
float i = 0;
extern float ACCX,ACCY,ACCZ;

int add = 0;
int times = 0;

typedef struct struct_message
{
  uint8_t header;
  uint8_t parity;
  uint8_t data[6];
  uint8_t footer;
} DataPacket;

DataPacket DataRe;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rcv_buf[64] = {0};
char mpu_buff[64];
int rcv_flag = 0;

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
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 
	HAL_UART_Receive_DMA(&huart1, rcv_buf, 8);	
	
	AS5048_init(1,&hspi1,GPIOA,GPIO_PIN_4);
	AS5048_init(2,&hspi2,GPIOB,GPIO_PIN_12);
	
	mpu_data[0].cali = 1;
	mpu_data[0].vel[0] = 0;
	mpu_data[0].vel[1] = 0;
  mpu_data[0].REAL_YAW_SET = 0;
	mpu_data[0].REAL_YAW_MARK = 0;
		
	can_filter_init();
	IM_TEST_initialize();
	
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim11);
	
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

//DMA+空闲中断 用于接收上位机信息
void Rcv_IdleCallback(void){
	//判断空闲中断发生
	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == SET){
		//清除空闲中断标志位，暂停串口DMA传输
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		HAL_UART_DMAStop(&huart1);
		//接收完成标志位
		rcv_flag = 1;
	}
}

int Rcv_DealData(void){
	if(1==rcv_flag){
		//数据处理
		if(0x0F==rcv_buf[0]&&0xAA==rcv_buf[7]){
			//编码轮强制更新指令
			DATARELOAD(rcv_buf);
		}else if(0xBB==rcv_buf[0]&&0xCC==rcv_buf[7]){
			//陀螺仪硬件复位指令
			HAL_GPIO_WritePin(RST_CTRL_GPIO_Port,RST_CTRL_Pin,GPIO_PIN_SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(RST_CTRL_GPIO_Port,RST_CTRL_Pin,GPIO_PIN_RESET);
			DATARELOAD(rcv_buf);
		}
		for(int i=0;i<8;i++){
			rcv_buf[i]=0;
		}
		//恢复标志位
		rcv_flag = 0;
		//发起下一次的串口DMA接收
		HAL_UART_Receive_DMA(&huart1, rcv_buf, 8);
		return 0;
	}else{
		return -1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim14)){
			
    }
		
		if (htim == (&htim13)){
			
		}
		 
		if (htim == (&htim11)){
			if(mpu_data[0].cali == 1){
				if(times >= 500){
						add++;
						AS5048_getREGValue(1);
						AS5048_dataUpdate(1);	
						AS5048_getREGValue(2);
						AS5048_dataUpdate(2);	
						
						mpu_data[0].REAL_YAW = mpu_data[0].YAW_ANGLE;

            rtU.W1 = -AS5048s[1].delta_dis;
            rtU.W2 = AS5048s[0].delta_dis;
            rtU.DEG = mpu_data[0].REAL_YAW;
						
            mpu_data[0].Y_tt += rtY.YOUT ;//*0.014373;
            mpu_data[0].X_tt += rtY.XOUT ;//*0.014373;
						mpu_data[0].REAL_Y = mpu_data[0].Y_tt * 0.014373;
						mpu_data[0].REAL_X = mpu_data[0].X_tt * 0.014373;
						// x y yaw
						
						Rcv_DealData();
						
						if(add >= 50){
							memset(mpu_buff, 0, 64);
							//bc为与上位机握手标识
							int mpu_len = sprintf(mpu_buff,"bc %f %f %f %f\r\n",mpu_data[0].REAL_X,mpu_data[0].REAL_Y,mpu_data[0].REAL_YAW,mpu_data[0].ROLL_ANGLE);
							HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&mpu_buff, mpu_len);
						  add = 0;
						}
					}else{
						
							AS5048_getREGValue(1);
							AS5048_dataUpdate(1);	
							AS5048_getREGValue(2);
							AS5048_dataUpdate(2);
					  mpu_data[0].vel[0] = 0;
				    mpu_data[0].vel[1] = 0;
						times ++ ;
					}
				IM_TEST_step();
			}
		}
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
