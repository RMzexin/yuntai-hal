#include "RemoteTask.h"	
#include "PIDcontrolTask.h"
#include "CanBusTask.h"
#include "DataProcessingTask.h"
#include "FireTask.h"
#include "pid.h"
#include "cmsis_os.h"
#include <math.h>

extern osTimerId CanTimerSendHandle;
Gimbal_Ref_t gimbal_ref;				//��̨����

PID_t Position_pitchPID;        //pitchλ�û�PID�������
PID_t Position_yawPID;          //yawλ�û�PID�������
PID_t Position_pluckPID;          //������λ�û�PID�������
PID_t Speed_pitchPID;           //pitch�ٶȻ�PID�������
PID_t Speed_yawPID;             //yaw�ٶȻ�PID�������
PID_t Speed_pluckPID;             //�������ٶȻ�PID�������

PID_value_t PID_pitchPosition_value;  //pitchλ�û�����ֵ ʵ��ֵ
PID_value_t PID_yawPosition_value;    //yawλ�û�����ֵ ʵ��ֵ
PID_value_t PID_pluckPosition_value;    //������λ�û�����ֵ ʵ��ֵ
PID_value_t PID_pitchSpeed_value;     //pitch�ٶȻ�����ֵ ʵ��ֵ
PID_value_t PID_yawSpeed_value;       //yaw�ٶȻ�����ֵ ʵ��ֵ
PID_value_t PID_pluckSpeed_value;       //�������ٶȻ�����ֵ ʵ��ֵ
PID_value_t PID_CM1_value; 
PID_value_t PID_CM2_value;
PID_value_t PID_CM3_value;
PID_value_t PID_CM4_value;

pid_t CM1pid={100,3.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000};
pid_t CM2pid={100,3.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000}; 
pid_t CM3pid={100,3.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000}; 
pid_t CM4pid={100,3.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000}; 
pid_t   YAWPpid={4.665,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000};
pid_t PITCHPpid={3.85,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000}; 
pid_t   YAWSpid={64.75,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000}; 
pid_t PITCHSpid={62.75,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000}; 
pid_t PLUCKPpid={1.2,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000}; 
pid_t PLUCKSpid={1.4,0.12,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,10000,-10000}; 

PID_Angle_Speed_t PID_Angle_Speed;

void Chassis_And_Gimbal_Data_Init(void)
{
	gimbal_ref.pitch_angle_dynamic_ref = +GMPitchEncoder.ecd_angle;		//����̨�ȶ����ϵ��λ��
  gimbal_ref.yaw_angle_dynamic_ref   = +GMYawEncoder.ecd_angle;
	gimbal_ref.pluck_angle_dynamic_ref = +GMPluckEncoder.ecd_angle;
	
	PID_pitchPosition_value.ideal  = gimbal_ref.pitch_angle_dynamic_ref;
	PID_pitchPosition_value.actual = GMPitchEncoder.ecd_angle;
	PID_yawPosition_value.ideal    = gimbal_ref.yaw_angle_dynamic_ref;
	PID_yawPosition_value.actual   = GMYawEncoder.ecd_angle;
	PID_pluckPosition_value.ideal  = gimbal_ref.pluck_angle_dynamic_ref;
	PID_pluckPosition_value.actual = GMPluckEncoder.ecd_angle;
}


                   
void Set_Gimbal_Motor_Output(void)	 //can ���
{	
 	Set_Gimbal_Current(&hcan1, (int16_t)YAWSpid.pidout, (int16_t)PITCHSpid.pidout, (int16_t)PLUCKSpid.pidout);
 // Set_Gimbal_Current(&hcan1, (int16_t)0, (int16_t)0, (int16_t)0);	
	Set_CM_Speed(&hcan1,(int16_t)CM1pid.pidout,(int16_t)CM2pid.pidout,(int16_t)CM3pid.pidout,(int16_t)CM4pid.pidout);
}


void Gimbal_RC_Mode(void)          //ң��ֵд��
{
//	  mpu9250_assignment();// �����Ǹ�ֵ
	  gimbal_ref.pitch_angle_dynamic_ref -=((Get_ch3_Data()-1024)*STICK_TO_PITCH_ANGLE_INC_FACT) ;
    gimbal_ref.yaw_angle_dynamic_ref   -=((Get_ch2_Data()-1024)*STICK_TO_YAW_ANGLE_INC_FACT);
	  gimbal_ref.pluck_angle_dynamic_ref -=PLUCK();
		gimbal_ref.left_right_ref           =((Get_ch0_Data()-1024)*STICK_TO_FORWARD_BACK_ANGLE_INC_FACT) ;
    gimbal_ref.forward_back_ref         =((Get_ch1_Data()-1024)*STICK_TO_LEFT_RIGHT_ANGLE_INC_FACT);
	  
	
	  PID_CM1_value.ideal = -gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f ;
	  PID_CM2_value.ideal = gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f;
	  PID_CM3_value.ideal = -gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f;
	  PID_CM4_value.ideal = gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f;
	  PID_pitchPosition_value.ideal = gimbal_ref.pitch_angle_dynamic_ref;
	  PID_yawPosition_value.ideal   = gimbal_ref.yaw_angle_dynamic_ref;
	  PID_pluckPosition_value.ideal = gimbal_ref.pluck_angle_dynamic_ref;
}


void PID_calculate_position_self(void)//��̨λ�û�PID�㷨
{ 
	pid_calculate(&PITCHPpid,GMPitchEncoder.ecd_angle,PID_pitchPosition_value.ideal);	
	pid_calculate(&YAWPpid,  GMYawEncoder.ecd_angle,  PID_yawPosition_value.ideal);	
	pid_calculate(&PLUCKPpid,GMPluckEncoder.ecd_angle,PID_pluckPosition_value.ideal);	
  PID_calculate_speed_self();	
}

void YUNTAI_SPEED()
{
	Yuntai_angle_speed(&PID_pitchSpeed_value.actual,&PID_yawSpeed_value.actual,&PID_pluckSpeed_value.actual);
}

void PID_calculate_speed_self(void)//��̨�ٶȻ�PID�㷨
{			
	PID_pitchSpeed_value.ideal =PITCHPpid.pidout;
	PID_yawSpeed_value.ideal   =YAWPpid.pidout;
	PID_pluckSpeed_value.ideal =PLUCKPpid.pidout;
	pid_calculate(&PITCHSpid,-PID_pitchSpeed_value.actual,-PID_pitchSpeed_value.ideal);
	pid_calculate(&YAWSpid,  -PID_yawSpeed_value.actual,  -PID_yawSpeed_value.ideal);
  pid_calculate(&PLUCKSpid,PID_pluckSpeed_value.actual ,PID_pluckSpeed_value.ideal);	
  Set_Gimbal_Motor_Output();	
}

void PID_calculate_chassis_self(void)//����PID�㷨
{
  pid_calculate(&CM1pid,CM1Encoder.rotor_speed,-PID_CM1_value.ideal);	
	pid_calculate(&CM2pid,CM2Encoder.rotor_speed,-PID_CM2_value.ideal);	
	pid_calculate(&CM3pid,CM3Encoder.rotor_speed,-PID_CM3_value.ideal);	
	pid_calculate(&CM4pid,CM4Encoder.rotor_speed,-PID_CM4_value.ideal);	
	Set_Gimbal_Motor_Output();
}

