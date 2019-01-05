#include "sprintf.h"
#include "mpu9250.h"
#include "pid.h"
#include "usart.h"
#include "RemoteTask.h"
#include "CanBusTask.h"
#include "PIDcontrolTask.h"


unsigned char DataScope_OutPut_Buffer[42] = {0};

extern RC_Type RC_Ctl;

extern Gimbal_Ref_t gimbal_ref;				//云台数据
extern mpu9250_t mpu9250;

extern pid_t CM1pid;
extern pid_t CM2pid;
extern pid_t CM3pid;
extern pid_t CM4pid;
extern pid_t PLUCKSpid;
extern pid_t PLUCKPpid;

extern PID_value_t PID_pitchPosition_value;  //pitch位置环理想值 实际值
extern PID_value_t PID_yawPosition_value;    //yaw位置环理想值 实际值
extern PID_value_t PID_pluckPosition_value;    //拨弹位置环理想值 实际值
extern PID_value_t PID_pitchSpeed_value;     //pitch速度环理想值 实际值
extern PID_value_t PID_yawSpeed_value;       //yaw速度环理想值 实际值
extern PID_value_t PID_pluckSpeed_value;       //拨弹速度环理想值 实际值
extern PID_value_t PID_CM1_value;
extern PID_value_t PID_CM2_value;
extern PID_value_t PID_CM3_value;
extern PID_value_t PID_CM4_value;


extern PID_Angle_Speed_t PID_Angle_Speed;
extern PID_Angle_Speed_t PID_Angle_Speed;





void Print_PID_Data(void)
{                                                                             
     printf("A%6.3f\r\nB%6.3f\r\nC%6.3f\r\nD%6.3f\r\n",
		 (float)mpu9250.stcAngleX,
		 (float)mpu9250.stcAngleY,
		 (float)mpu9250.stcAngleZ,
		 (float)GMPitchEncoder.ecd_angle);
}

//该.c文件主要是配合山外调试助手发送波形
short  wave_form_data[6] = {0};
void send_data(uint8_t date)
{
	HAL_UART_Transmit(&huart3,&date,1,10);
	//while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);  
	
}
void shanwai_send_wave_form(void)
{
	uint8_t i;
	
	send_data(0x03);
	send_data(0xfc);
	for(i = 0;i<6;i++)
	{
	  send_data((wave_form_data[i]&0xff)); //现发送低位在发送高位
//	  send_data((wave_form_data[i]>>8));
//		send_data((wave_form_data[i]>>16));
//		send_data((wave_form_data[i]>>24));
		
	  
	}
	send_data(0xfc);
	send_data(0x03);
}	
void shanwai_sprintf()
{
		wave_form_data[0] =(float)PID_yawPosition_value.ideal;
	  wave_form_data[1] =(float)GMYawEncoder.ecd_angle;
	  wave_form_data[2] =(float)PID_pitchPosition_value.ideal;
	  wave_form_data[3] =(float)GMPitchEncoder.ecd_angle;
		wave_form_data[4] =(float)mpu9250.stcAngleZ;
	  wave_form_data[5] =(float)mpu9250.stcAngleY;
		shanwai_send_wave_form();   //将数据传输到三外上位机，可以看到实时波形
}

void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //??float???
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return; 
  else
  {
     switch (Channel)
		{
      case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
  }	 
}

unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }
  else
  {	
	 DataScope_OutPut_Buffer[0] = '$';
		
	 switch(Channel_Number)   
   { 
		 case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6; break;   
		 case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10; break;
		 case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; break;
		 case 4:   DataScope_OutPut_Buffer[17] = 17; return 18; break;
		 case 5:   DataScope_OutPut_Buffer[21] = 21; return 22; break; 
		 case 6:   DataScope_OutPut_Buffer[25] = 25; return 26; break;
		 case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; break;
		 case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; break;
		 case 9:   DataScope_OutPut_Buffer[37] = 37; return 38; break;
     case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; break;
   }	 
  }
	return 0;
}








