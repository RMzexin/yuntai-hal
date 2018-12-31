#ifndef _PID_H_
#define _PID_H_
#include "stm32f4xx_hal.h"

//PID�ṹ������
enum{
    LLAST	= 2,
    LAST 	= 1,
    NOW 	= 0,
    
    POSITION_PID,
    DELTA_PID,
};

typedef struct 
	{
		float p;
		float i;
		float d;
    
		float set[3];				//Ŀ��ֵ,����NOW�� LAST�� LLAST���ϴ�
		float get[3];				//����ֵ
    float err[3];				//���
	
	  float Voltage;     //������
	  float Voltage_max;     //��������
	  float Voltage_min;     //��С������
	
    float pout;							//p���
    float iout;							//i���
    float dout;							//d���
	  float pidout;
		float MaxOutput;				//����޷�
		float MinOutput;	
		
		
}pid_t;
	

float pid_calculate(pid_t* pid, float get, float set);

#endif






