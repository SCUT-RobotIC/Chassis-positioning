#ifndef YIS130_H
#define YIS130_H
#include <stdint.h>
#define MPU_CAN   hcan1   


typedef struct
{
    float acc[3];
		float acc_cali[3];
		double vel[3];
	  float gyro[3];         
    float PITCH;
	  float YAW; 
	  float ROLL;
	  float PITCH_ANGLE;
	  float YAW_ANGLE; 
	  float ROLL_ANGLE;
		float quat[4];
	  float ACCX_CALI;
		float ACCY_CALI;
		float ACCZ_CALI;
	float ACCX_FILTER;
	float ACCY_FILTER;
		
	int cali ;
	
	
} MPU_DATA;




void CAN_CMD_ENCODER();


void can_filter_init(void);

void get_mpu_measure(MPU_DATA *ptr,uint8_t *data ) ;

extern MPU_DATA mpu_data[4];
extern float output_vector_data[3];

void VECTOR_CONVERT();

void SelfCalibration();

extern float ACCX,ACCY,ACCZ; // ��ߵ�У׼�����Ҫ�ϳ����ܲ���
void arm_fir_f32_lp(void);

#endif
