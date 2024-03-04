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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BRT_encoder.h"
#include "MPU6050.h"
#include "arm_math.h"
#include "IM_TEST.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t ID ;
uint32_t STD ;
int i = 0;

int x = 0;
int x_start = 0;
int y = 0;
int y_start = 0;

float x_mm = 0;
float y_mm = 0;
float tt_y = 0;
float tt_x = 0;

float test_x = 0;
float test_y = 0;
float test_deg = 0 ;

float a =0;

double DATA_upload[12]={0}; // x_pos >> y_pos >> x_ori >> y_ori >> z_ori >> w_ori >> x_ang >> y_ang >> z_ang >> x_acc >> y_acc >> z_acc;
extern float q0 ,q1,q2,q3;

extern encoder_data_t encoder_data[4];



MPU6050_t Mpu6050;
#define FIVE_MS_ERROR   0.00002115 // ���ϵ�Ư������ 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int fputc(int ch, FILE *f)
 
{
 
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
 
  return ch;
 
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	
	MPU6050_Init(&hi2c1); 
	DMP_Init(&hi2c1); 
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_Base_Start_IT(&htim14);
	
	can_filter_init();


	ID = 0x03;
	STD = 0x003;
	CAN_CMD_ENCODER(ID,STD);
	HAL_Delay(1);
	ID = 0x01;
	STD = 0x001;
	CAN_CMD_ENCODER(ID,STD);

	HAL_Delay(1);

	x_start = encoder_data[0].tt_ecd;
	encoder_data[0].last_tt_ecd = x_start;
	y_start = encoder_data[2].tt_ecd;
	encoder_data[2].last_tt_ecd = y_start;
	
	Mpu6050.ac_error = 0;	
		
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // BRT 1024
	
    if (htim == (&htim14))
    {
			
			

    }
		
		if (htim == (&htim13)){
			 		 
			Mpu6050.ac_error += FIVE_MS_ERROR ;

		 
		 }
		 
		 if (htim == (&htim11)){
			
			 		
					ID = 0x03;
					STD = 0x003;
					CAN_CMD_ENCODER(ID,STD);
					x = encoder_data[0].tt_ecd-encoder_data[0].last_tt_ecd;
					encoder_data[0].last_tt_ecd = encoder_data[0].tt_ecd;
					x_mm = x  ;  // wrong
				
					HAL_Delay(1);
					
					ID = 0x01;
					STD = 0x001;
					CAN_CMD_ENCODER(ID,STD);
					
				
					y = encoder_data[2].tt_ecd-encoder_data[2].last_tt_ecd;
					encoder_data[2].last_tt_ecd = encoder_data[2].tt_ecd;
			
					y_mm = y ;
					
					HAL_Delay(1);
						
					MPU6050_UPDATE(&hi2c1,&Mpu6050);	
						
					rtU.W1 = x_mm;
					rtU.W2 = y_mm;
					rtU.DEG = Mpu6050.Yaw;
						 
				  tt_y += rtY.YOUT;
					tt_x += rtY.XOUT;


          DATA_upload[0] = tt_x;
          DATA_upload[1] = tt_y;
          DATA_upload[2] = q0;
          DATA_upload[3] = q1;
          DATA_upload[4] = q2;
          DATA_upload[5] = q3;
          DATA_upload[6] = Mpu6050.Gx;
          DATA_upload[7] = Mpu6050.Gy;
          DATA_upload[8] = Mpu6050.Gy;
          DATA_upload[9] = Mpu6050.Ax;
          DATA_upload[10] = Mpu6050.Ay;
          DATA_upload[11] = Mpu6050.Az;

					i++;
					
					if( i ==100){
					
				//	printf("%f %f %f\r\n",tt_y,tt_x,Mpu6050.Yaw); 
				
            HAL_UART_Transmit(&huart1, (uint8_t *)DATA_upload, 96, 0xffff);
					
						i = 0 ;
					}
					
					IM_TEST_step();
        
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
