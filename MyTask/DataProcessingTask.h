#ifndef _DATA_PROCESSING_TASK_H_
#define _DATA_PROCESSING_TASK_H_

#include "CanBusTask.h"
#include "stm32f4xx_hal.h"
#include "PIDcontrolTask.h"

void Yuntai_angle_speed(float *pitch_speed , float *yaw_speed, float *pluck_speed);
	
#endif

