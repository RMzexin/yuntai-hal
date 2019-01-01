#ifndef __BSP_CAN
#define __BSP_CAN

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#elif defined STM32F1
#include "stm32f1xx_hal.h"
#endif
#include "can.h"
#include "stm32f4xx_hal.h"
#include "PIDcontrolTask.h"
#include "DataProcessingTask.h"

/*CAN���ͻ��ǽ��յ�ID*/
/* CAN Bus 1 */  
#define CAN_BUS1_ZGYRO_FEEDBACK_MSG_ID            0x401
/* CAN Bus 2 */  
#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID           0x205
#define CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID           0x206
#define CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID           0x207


#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                    //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								    //�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	    //buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;							    //�ٶ�
	float ecd_angle;											//�Ƕ�
	float rotor_speed;                    //ת���ٶ�
}Encoder;

typedef enum
{
	false,
	true
}Bool;

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder GMPluckEncoder;

float Yaw_Angle_Precision_Filter(volatile Encoder *Y);
float Pitch_Angle_Precision_Filter(volatile Encoder *P);
float Pluck_Angle_Precision_Filter(volatile Encoder *L);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void GetEncoderBias(volatile Encoder *v, CAN_HandleTypeDef * msg);
void EncoderProcess(volatile Encoder *v, CAN_HandleTypeDef * msg);
void CanReceiveMsgProcess(CAN_HandleTypeDef * msg);
void Set_CM_Speed(CAN_HandleTypeDef* hcan, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void Set_Gimbal_Current(CAN_HandleTypeDef* hcan, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_pluck_iq);
#endif

