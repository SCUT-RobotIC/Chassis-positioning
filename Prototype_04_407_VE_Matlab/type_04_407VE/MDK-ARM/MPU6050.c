
#include "MPU6050.h"
#include "i2c.h"

uint8_t check;

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f
#define FIVE_MS_ERROR   0.00002115 // ���ϵ�Ư������
short gyro[3], accel[3], sensors;
float Pitch,Roll,Yaw; 
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;            // error
    return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x03) {                   //����0x03ΪMPU6050
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);			//��ȡ��ǰ�����ǵ�״̬
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);			//���ݶ�ȡ��״̬����У׼
		
        mpu_get_accel_sens(&accel_sens);	//��ȡ��ǰ���ٶȼƵ�״̬
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);			//���ݶ�ȡ��״̬����У׼
		//printf("setting bias succesfully ......\r\n");
    }
}

uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];

int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*��������:	    ���µ�ADC���ݸ��µ� FIFO���飬�����˲�����

* �ƺ�û�� ������simulink��ʵ�����ƹ���
*******************************************************************************/

void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	
	
	unsigned char i ;
	int32_t sum=0;
	for(i=1;i<10;i++){	//FIFO ����
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;

	sum=0;
	for(i=0;i<10;i++){	//��ǰ����ĺϣ���ȡƽ��ֵ
	   sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
}

void MPU6050_UPDATE(I2C_HandleTypeDef *I2Cx,MPU6050_t *DataStruct)
{		
		Read_DMP();
	
//	
//    uint8_t Rec_Data[6];


//    // Read 6 BYTES of data starting from ACCEL_XOUT_H register
//	
//		
//    HAL_I2C_Mem_Read(I2Cx, devAddr, MPU6050_RA_ACCEL_XOUT_H, 1, Rec_Data, 6, 100);

//    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
//    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
//    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

//    /*** convert the RAW values into acceleration in 'g'
//         we have to divide according to the Full scale value set in FS_SEL
//         I have configured FS_SEL = 0. So I am dividing by 16384.0
//         for more details check ACCEL_CONFIG Register              ****/

//    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
//    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
//    DataStruct->Az = DataStruct->Accel_Z_RAW / 14418.0; 
//	 //��λΪ g �� 9.8


		MPU6050_newValues(accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);

    //gyro[3]   accel[3], sensors;	
		DataStruct->Ax = MPU6050_FIFO[0][10] *0.00059814453125          ;
		DataStruct->Ay = MPU6050_FIFO[1][10] *0.00059814453125          ;
		DataStruct->Az = (MPU6050_FIFO[2][10] -16384 )*0.00059814453125 ;
		DataStruct->Gx = MPU6050_FIFO[3][10] / 16.4;
		DataStruct->Gy = MPU6050_FIFO[4][10] /16.4 ;
		DataStruct->Gz = MPU6050_FIFO[5][10] /16.4;
		
	
    	DataStruct->ac_error += FIVE_MS_ERROR;
		DataStruct->Roll = Roll + DataStruct->ac_error;
		DataStruct->Yaw = Yaw + DataStruct->ac_error;
		DataStruct->Pitch = Pitch+ DataStruct->ac_error;
		
}
void MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, devAddr , MPU6050_RA_WHO_AM_I, 1, &check, 1, 100);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
				Data = 0;
        HAL_I2C_Mem_Write(I2Cx, devAddr ,MPU6050_RA_PWR_MGMT_1, 1, &Data, 1, 100);
				
				// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, devAddr, MPU6050_RA_SMPLRT_DIV, 1, &Data, 1, 100);
			
        // full-scale gyroscope range.
				// �� 2000 ��/s 00011000
				Data = 0x18 ;
        HAL_I2C_Mem_Write(I2Cx, devAddr , MPU6050_RA_GYRO_CONFIG, 1, &Data, 1, 100);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx,devAddr , MPU6050_RA_ACCEL_CONFIG, 1, &Data, 1, 100);

        
    }
    
}

void DMP_Init(I2C_HandleTypeDef *I2Cx)
{ 
//	uint8_t temp[1]={0};
	HAL_I2C_Mem_Read(I2Cx, devAddr , MPU6050_RA_WHO_AM_I, 1, &check, 1, 100);
	
	//printf("mpu_set_sensor complete ......\r\n");
	if(check!=0x68)NVIC_SystemReset();
	
	if(!mpu_init())
	{
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
			//printf("mpu_set_sensor complete ......\r\n");
		}
		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
			//printf("mpu_configure_fifo complete ......\r\n");
		}
		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
		{
			//printf("mpu_set_sample_rate complete ......\r\n");
		}
		if(!dmp_load_motion_driver_firmware())
		{
			//printf("dmp_load_motion_driver_firmware complete ......\r\n");
		}
		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
		{
			//printf("dmp_set_orientation complete ......\r\n");
		}
		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		DMP_FEATURE_GYRO_CAL))
		{
			//printf("dmp_enable_feature complete ......\r\n");
		}
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
		{
			//printf("dmp_set_fifo_rate complete ......\r\n");
		}
		run_self_test();
		if(!mpu_set_dmp_state(1))
		{
			//printf("mpu_set_dmp_state complete ......\r\n");
		}
	}
}
/**************************************************************************
�������ܣ���ȡMPU6050����DMP����̬��Ϣ
��ڲ�������
����  ֵ����
**************************************************************************/
void Read_DMP(void)
{	
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		
	if (sensors & INV_WXYZ_QUAT )
	{    
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
     
		 Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	
		 Roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
		 Yaw = atan2(2 * (q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;//yaw
	}
}
///**************************************************************************
//�������ܣ���ȡMPU6050�����¶ȴ���������
//��ڲ�������
//����  ֵ�������¶�
//**************************************************************************/
//int Read_Temperature(void)
//{	   
//	float Temp;
//	Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
//	
//	
//	if(Temp>32768) Temp-=65536;
//	Temp=(36.53+Temp/340)*10;
//	return (int)Temp;
//}

/**************************************************************************
�������ܣ���ȡ�Ƕ� 0-359
��ڲ�������
����  ֵ����
**************************************************************************/
void getAngle_YAW(float *yaw,float *yaw_acc_error)
{
	Read_DMP();                  
	
	if(Yaw < 0)
		Yaw = Yaw + 360;
	*yaw = Yaw;                   
	
	*yaw = *yaw - *yaw_acc_error; 
	
	if(*yaw < 0)
		*yaw = *yaw+360;
}



void getAngle_ROLL(float *roll,float *roll_acc_error)
{
	Read_DMP();                   
	
	if(Roll < 0)
		Roll = Roll + 360;
	*roll = Roll;                   
	
	*roll = *roll - *roll_acc_error; 
	
	if(*roll < 0)
		*roll = *roll+360;
}

void getAngle_PITCH(float *pitch,float *pitch_acc_error)
{
	Read_DMP();                   
	
	if(Pitch < 0)
		Pitch = Pitch + 360;
	*pitch = Pitch;                  
	
	*pitch = *pitch - *pitch_acc_error; 
	
	if(*pitch < 0)
		*pitch = *pitch+360;
}
// ��HAL��ʵ�ֵ�i2cWrite����
// �����ͷ���ֵ��ԭ������ͬ
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    
    if (HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}


// ��HAL��ʵ�ֵ�i2cRead����
// �����ͷ���ֵ��ԭ������ͬ
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    
    if (HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}


//------------------End of File----------------------------
