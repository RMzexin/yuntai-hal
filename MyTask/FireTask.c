#include "FireTask.h"
#include "CanBusTask.h"
#include "tim.h"
extern RC_Type RC_Ctl;

//����Ħ���ֵ���ź�
void  FireSetSingle()          
{
motor_pwm_setvalue1(1000);
}


//��������ź�
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
//����
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
	


