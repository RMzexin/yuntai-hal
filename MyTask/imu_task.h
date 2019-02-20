#ifndef _IMU_TASK_H
#define _IMU_TASK_H
#include "stm32f4xx_hal.h"

typedef struct
{
	float pitch;
	float yaw;
  float roll;
	
	short aacx;
	short aacy;
	short aacz;
	
	short gyrox;
	short gyroy;
	short gyroz;
	
	short temp;	
}IMU_t;				//云台数据（期望值）结构体

void Get_IMU_data(void);
#endif


