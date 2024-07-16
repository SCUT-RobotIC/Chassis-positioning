#include "YIS130.h"
#include "main.h"

#include "arm_math.h"


#define TEST_LENGTH_SAMPLES  128  
#define BLOCK_SIZE           1        
#define NUM_TAPS             29     

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;    

static float32_t testInput_f32_50Hz_200Hz_X[TEST_LENGTH_SAMPLES]; /* 采样点 */
static float32_t testInput_f32_50Hz_200Hz_Y[TEST_LENGTH_SAMPLES]; /* 采样点 */
static float32_t testOutput_X[TEST_LENGTH_SAMPLES];               /* 滤波后的输出 */
static float32_t testOutput_Y[TEST_LENGTH_SAMPLES];               /* 滤波后的输出 */
static float32_t firStateF32_X[BLOCK_SIZE + NUM_TAPS - 1];        /* 状态缓存，大小numTaps + blockSize - 1*/
static float32_t firStateF32_Y[BLOCK_SIZE + NUM_TAPS - 1];        /* 状态缓存，大小numTaps + blockSize - 1*/



/* 低通滤波器系数 通过fadtool获取*/
const float32_t firCoeffs32LP[NUM_TAPS] = {
  -0.001822523074f,  -0.001587929321f,  1.226008847e-18f,  0.003697750857f,  0.008075430058f,
  0.008530221879f,   -4.273456581e-18f, -0.01739769801f,   -0.03414586186f,  -0.03335915506f,
  8.073562366e-18f,  0.06763084233f,    0.1522061825f,     0.2229246944f,    0.2504960895f,
  0.2229246944f,     0.1522061825f,     0.06763084233f,    8.073562366e-18f, -0.03335915506f,
  -0.03414586186f,   -0.01739769801f,   -4.273456581e-18f, 0.008530221879f,  0.008075430058f,
  0.003697750857f,   1.226008847e-18f,  -0.001587929321f,  -0.001822523074f
};



extern CAN_HandleTypeDef    hcan1;//set can
int ecd_gb = 0;
MPU_DATA mpu_data[4];
float output_vector_data[3];
arm_matrix_instance_f32 output_vector; 
static CAN_TxHeaderTypeDef  encoder_tx_message;
static uint8_t         encoder_can_send_data[4];
float ACCX = 0; float ACCY = 0; float ACCZ = 0;

/// @brief actually not use 

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

}


/// @brief hal_CAN,MPU�

//本文件注释掉的代码为之前调试所用，可完全忽略
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];


    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
	  uint16_t pitch_raw;
	  uint16_t roll_raw; 
	  uint16_t yaw_raw;
    switch (rx_header.ExtId)
    {
        case 0x0CF02D59: //acc
            
//						mpu_data[0].acc[0] = 1; //X Y Z
//            mpu_data[0].acc[1] = 2;   
//            mpu_data[0].acc[2] = 3;

//            break;
            // Little-endian
						VECTOR_CONVERT();

            mpu_data[0].acc[0] = ((float)(rx_data[1] << 8 | rx_data[0]))* 0.01 -320; //X Y Z
            mpu_data[0].acc[1] = ((float)(rx_data[3] << 8 | rx_data[2]))* 0.01 -320;   
            mpu_data[0].acc[2] = ((float)(rx_data[5] << 8 | rx_data[4]))* 0.01 -320;
					
//						mpu_data[0].acc_cali[0] = mpu_data[0].acc[0] - mpu_data[0].ACCX_CALI;
//            mpu_data[0].acc_cali[1] = mpu_data[0].acc[1] - mpu_data[0].ACCY_CALI;
//            mpu_data[0].acc_cali[2] = mpu_data[0].acc[2] - mpu_data[0].ACCZ_CALI;

//            // 计算重力的影响
//            float gX = arm_sin_f32(mpu_data[0].ROLL) * arm_cos_f32(mpu_data[0].PITCH) * 9.81;
//            float gY = arm_cos_f32(mpu_data[0].ROLL) * arm_cos_f32(mpu_data[0].PITCH) * 9.81;
//            float gZ = arm_sin_f32(mpu_data[0].PITCH) * 9.81;

//						mpu_data[0].acc_cali[0] = mpu_data[0].acc[1] - gX;
//            mpu_data[0].acc_cali[1] = mpu_data[0].acc[2] - gY;
//            mpu_data[0].acc_cali[2] = mpu_data[0].acc[0] + gZ;
						
						mpu_data[0].acc_cali[0] = mpu_data[0].acc[0] - output_vector_data[1];
            mpu_data[0].acc_cali[1] = mpu_data[0].acc[1] + output_vector_data[0];
            mpu_data[0].acc_cali[2] = mpu_data[0].acc[2] + output_vector_data[2];
					  ACCX = mpu_data[0].acc[0];
						ACCY = mpu_data[0].acc[1];
						ACCZ = mpu_data[0].acc[2];

				
            break;
        case 0x0CF02A59: // gryo unsign 20 bit each 
            // EACH 20 bit			
            // ignore
							
						// 提取 GYRO_X
						 mpu_data[0].gyro[0] = (float)((((uint32_t)(rx_data[2]&0x0F) << 16) | ((uint32_t)rx_data[1] << 8) | rx_data[0])     )* 0.0078125 - 4000;
						// 提取 GYRO_Y
						 mpu_data[0].gyro[1]=  (float)(((uint32_t)(rx_data[4])<< 12) | ((uint32_t)rx_data[3] << 4) | ((rx_data[2]&0xF0)>> 4))* 0.0078125 - 4000;
						// 提取 GYRO_Z
						 mpu_data[0].gyro[2] = (float)((((uint32_t)(rx_data[7]&0x0F) << 16) | ((uint32_t)rx_data[6] << 8) | rx_data[5] ))    * 0.0078125 - 4000;
				    break;
				
				case 0x0CF03059: // quat
            // Little-endian
            mpu_data[0].quat[0] = ((float)(((rx_data[1] << 8) | rx_data[0])))* 3.0519E-005 - 1;  // w
            mpu_data[0].quat[1] = ((float)(((rx_data[3] << 8) | rx_data[2])))* 3.0519E-005 - 1;    // x
            mpu_data[0].quat[2] = ((float)(((rx_data[5] << 8) | rx_data[4])))* 3.0519E-005 - 1;    //y  
						mpu_data[0].quat[3] = ((float)(((rx_data[7] << 8) | rx_data[6])))* 3.0519E-005 - 1; // z
            break;
				

            break;
        case 0x0CF02959: // pitch yaw roll
            // Little-endian
//						mpu_data[0].PITCH = 0.5; // Y
//            mpu_data[0].ROLL = 0.4; // X
//            mpu_data[0].YAW = 0.3; //Z
//            break;

					 pitch_raw= ((uint16_t)rx_data[1] << 8) | rx_data[0];
					 roll_raw = ((uint16_t)rx_data[3] << 8) | rx_data[2];
					 yaw_raw = ((uint16_t)rx_data[5] << 8) | rx_data[4];

            mpu_data[0].PITCH_ANGLE =  (float)pitch_raw * 0.0078125 - 250 ; // Y
            mpu_data[0].ROLL_ANGLE = (float)roll_raw * 0.0078125 - 250; // X
            mpu_data[0].YAW_ANGLE= (float)yaw_raw * 0.0078125 - 250; //Z
				
//			    mpu_data[0].PITCH_ANGLE =  asin(-2 * mpu_data[0].quat[1] * mpu_data[0].quat[3] + 2 * mpu_data[0].quat[0]* mpu_data[0].quat[2])* 57.3; 	
//					mpu_data[0].ROLL_ANGLE = atan2(2 * mpu_data[0].quat[2] * mpu_data[0].quat[3] + 2 * mpu_data[0].quat[0] * mpu_data[0].quat[1], -2 * mpu_data[0].quat[1] * mpu_data[0].quat[1] - 2 * mpu_data[0].quat[2]* mpu_data[0].quat[2] + 1)* 57.3; 
//					mpu_data[0].YAW_ANGLE= atan2(2 * (mpu_data[0].quat[1]*mpu_data[0].quat[2] + mpu_data[0].quat[0]*mpu_data[0].quat[3]),mpu_data[0].quat[0]*mpu_data[0].quat[0]+mpu_data[0].quat[1]*mpu_data[0].quat[1]-mpu_data[0].quat[2]*mpu_data[0].quat[2]-mpu_data[0].quat[3]*mpu_data[0].quat[3])*57.3;//yaw
//			
				    mpu_data[0].PITCH = mpu_data[0].PITCH_ANGLE * (3.1415926/180); // Y
            mpu_data[0].ROLL=  mpu_data[0].ROLL_ANGLE * (3.1415926/180); // X
            mpu_data[0].YAW = mpu_data[0].YAW_ANGLE * (3.1415926/180); //Z

//            mpu_data[0].PITCH_ANGLE_Del = mpu_data[0].PITCH_ANGLE - mpu_data[0].PITCH_ANGLE_BEG;
//            mpu_data[0].ROLL_ANGLE_Del = mpu_data[0].ROLL_ANGLE - mpu_data[0].ROLL_ANGLE_BEG;
//            mpu_data[0].YAW_ANGLE_Del = mpu_data[0].YAW_ANGLE - mpu_data[0].YAW_ANGLE_BEG;

				   break;
        
        default:
            break;
    }
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void VECTOR_CONVERT(){
  // input XYZ euler , ACC xyz ,
  // output ACC in world frame
	
		float32_t rotation_matrix_data[9] = {
		arm_cos_f32(mpu_data[0].YAW)*arm_cos_f32(mpu_data[0].PITCH), 
        arm_cos_f32(mpu_data[0].YAW)*arm_sin_f32(mpu_data[0].ROLL)*arm_sin_f32(mpu_data[0].PITCH)-arm_sin_f32(mpu_data[0].YAW)*arm_cos_f32(mpu_data[0].ROLL),
        arm_sin_f32(mpu_data[0].ROLL)*arm_sin_f32(mpu_data[0].YAW)+ arm_cos_f32(mpu_data[0].ROLL)*arm_cos_f32(mpu_data[0].YAW)*arm_sin_f32(mpu_data[0].PITCH),
        arm_sin_f32(mpu_data[0].YAW)*arm_cos_f32(mpu_data[0].PITCH),
        arm_sin_f32(mpu_data[0].YAW)*arm_sin_f32(mpu_data[0].PITCH)*arm_sin_f32(mpu_data[0].ROLL)+arm_cos_f32(mpu_data[0].YAW)*arm_cos_f32(mpu_data[0].ROLL),
        arm_sin_f32(mpu_data[0].YAW)*arm_sin_f32(mpu_data[0].PITCH)*arm_cos_f32(mpu_data[0].ROLL)-arm_cos_f32(mpu_data[0].YAW)*arm_sin_f32(mpu_data[0].ROLL),
        -arm_sin_f32(mpu_data[0].PITCH),
        arm_cos_f32(mpu_data[0].PITCH)*arm_sin_f32(mpu_data[0].ROLL),
        arm_cos_f32(mpu_data[0].PITCH)*arm_cos_f32(mpu_data[0].ROLL)
     };

		// rotation_matrix_data[8] = arm_cos_f32(mpu_data[0].PITCH)*arm_cos_f32(mpu_data[0].ROLL);
      
	    arm_matrix_instance_f32 rotation_matrix;
		  arm_matrix_instance_f32 trans_rotation_matrix;
      arm_mat_init_f32(&rotation_matrix, 3, 3, rotation_matrix_data);
			arm_mat_trans_f32(&rotation_matrix,&trans_rotation_matrix);
    
		float32_t GRA_vector_data[3] = {0, 0, - 9.7683};

    arm_matrix_instance_f32 GRA_vector;
    arm_mat_init_f32(&GRA_vector, 3, 1, GRA_vector_data);

    
    
    arm_mat_init_f32(&output_vector, 3, 1, output_vector_data);
    arm_mat_mult_f32(&rotation_matrix, &GRA_vector, &output_vector);
		
}

void arm_fir_f32_lp(void)
{
    uint32_t i;
    arm_fir_instance_f32 S_X;
		arm_fir_instance_f32 S_Y;
    float32_t  *inputF32_X, *outputF32_X;
    float32_t  *inputF32_Y, *outputF32_Y;
    /* 初始化输入输出缓存指针 */
    inputF32_X = &testInput_f32_50Hz_200Hz_X[0];
    outputF32_X = &testOutput_X[0];
	
		inputF32_Y = &testInput_f32_50Hz_200Hz_Y[0];
    outputF32_Y = &testOutput_Y[0];

    /* 初始化结构体S */
    arm_fir_init_f32(&S_X,                            
                     NUM_TAPS, 
                    (float32_t *)&firCoeffs32LP[0], 
                     &firStateF32_X[0], 
                     blockSize);
	
	  arm_fir_init_f32(&S_Y,                            
                     NUM_TAPS, 
                    (float32_t *)&firCoeffs32LP[0], 
                     &firStateF32_Y[0], 
                     blockSize);

    /*更新输入数据*/
    for(i=0; i < TEST_LENGTH_SAMPLES; i++)
    {
        testInput_f32_50Hz_200Hz_X[i] = mpu_data[0].acc[0];
				testInput_f32_50Hz_200Hz_Y[i] = mpu_data[0].acc[1];

    }

    /* 实现FIR滤波，这里每次处理1个点 */
    for(i=0; i < numBlocks; i++)
    {
        arm_fir_f32(&S_X, inputF32_X + (i * blockSize), outputF32_X + (i * blockSize), blockSize);
        arm_fir_f32(&S_Y, inputF32_Y + (i * blockSize), outputF32_Y + (i * blockSize), blockSize);
		}    

		mpu_data[0].ACCX_FILTER = testOutput_X[0];
    mpu_data[0].ACCY_FILTER = testOutput_Y[0];

}


//下面这个函数的办法不行，已弃用
void SelfCalibration(){
	mpu_data[0].cali =  0;
	int cali_times = 1000;
	while( cali_times >0){
			mpu_data[0].ACCX_CALI += mpu_data[0].acc[0];
			mpu_data[0].ACCY_CALI += mpu_data[0].acc[1];
			mpu_data[0].ACCZ_CALI += mpu_data[0].acc[2];
			HAL_Delay(5);
		 cali_times-- ;
	
	}
	mpu_data[0].ACCX_CALI = mpu_data[0].ACCX_CALI / 1000; 
	mpu_data[0].ACCY_CALI = mpu_data[0].ACCY_CALI / 1000; 
	mpu_data[0].ACCZ_CALI = mpu_data[0].ACCZ_CALI / 1000; 
	mpu_data[0].cali =  1;

}

void DATARELOAD(uint8_t * arr){
		uint16_t temp[2] = {0};
		
		if(arr[1]==0x00&&arr[2]==0x00){
			mpu_data[0].REAL_X = 0;
		}
		else if((arr[2]&0x80)==0x80)//负数
		{
			temp[0] = arr[2];
			temp[0] = temp[0] << 8;
			temp[0] += arr[1];
			temp[0] -= 1;
			temp[0] = ~temp[0];
			mpu_data[0].REAL_X = 0-temp[0];
		}else{
			mpu_data[0].REAL_X = (arr[1] | arr[2] << 8);
		}
		
		if(arr[3]==0x00&&arr[4]==0x00){
			mpu_data[0].REAL_Y = 0;
		}
		else if((arr[4]&0x80)==0x80)//负数
		{
			temp[1] = arr[4];
			temp[1] = temp[1] << 8;
			temp[1] += arr[3];
			temp[1] -= 1;
			temp[1] = ~temp[1];
			mpu_data[0].REAL_Y = 0-temp[1];
		}else{
			mpu_data[0].REAL_Y = (arr[3] | arr[4] << 8);
		}
		
    mpu_data[0].X_tt  = mpu_data[0].REAL_X / 0.014373;
    mpu_data[0].Y_tt  = mpu_data[0].REAL_Y / 0.014373;
		
}