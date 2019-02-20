#include "FireTask.h"
#include "CanBusTask.h"
#include "tim.h"
extern RC_Type RC_Ctl;

//配置摩擦轮电调信号
void  FireSetSingle()          
{
motor_pwm_setvalue1(1000);
}


//电调开启信号
void  FireOpen()            
{
motor_pwm_setvalue1(1645);  
}

void  Firefast()            
{
motor_pwm_setvalue1(1780);
}

float  PLUCK()
{
	static Bool first = true;
	if(RC_Ctl.switch_left==3&&first)
	{
		first = false;
		return 360.0*PLUCK_RATIO/9.0;
	}
	else if(RC_Ctl.switch_left==1&&first==false)
	{
	  first = true;
		return 0;
	}
	else
	{
		return 0;
	}
}
//拨弹
void  SHORT()
{
	if(RC_Ctl.switch_right==3)
	{
		FireOpen();
	}
	else if(RC_Ctl.switch_right==1)
	{
	  FireSetSingle();
	}
	else if(RC_Ctl.switch_right==2)
	{
		Firefast();
	}
}
	


