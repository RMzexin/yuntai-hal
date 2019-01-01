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

/*CAN发送或是接收的ID*/
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
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                    //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								    //滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	    //buf，for filter
	int32_t round_cnt;										//圈数
	int32_t filter_rate;							    //速度
	float ecd_angle;											//角度
	float rotor_speed;                    //转子速度
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

