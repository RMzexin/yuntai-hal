#include "RemoteTask.h"

RC_Type RC_Ctl;
uint32_t  Latest_Remote_Control_Pack_Time = 0;
uint32_t  LED_Flash_Timer_remote_control = 0;
/*******************************************************************************************
  * @Func		void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
  * @Brief  DR16���ջ�Э��������
  * @Param		RC_Type* rc���洢ң�������ݵĽṹ�塡��uint8_t* buff�����ڽ���Ļ���
  * @Retval		None
  * @Date    
 *******************************************************************************************/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
	rc->ch0 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =( (buff[5] >> 4)& 0x0003 ) ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.press_left 	= buff[12];	// is pressed?
	rc->mouse.press_right = buff[13];
	
	rc->keyBoard.key_code = buff[14] | buff[15] << 8; //key borad code
	
	Latest_Remote_Control_Pack_Time = HAL_GetTick();
	
	if(Latest_Remote_Control_Pack_Time - LED_Flash_Timer_remote_control>500){
			LED_Flash_Timer_remote_control = Latest_Remote_Control_Pack_Time;		
	}
}

void RC_Ctl_t_Init(void)	//��ң�����������г�ʼ��
{
	RC_Ctl.ch0 = 1024;		//ch0��ch1Ϊ��ҡ��
	RC_Ctl.ch1 = 1024;	
	RC_Ctl.ch2 = 1024;		//ch2��ch3Ϊ��ҡ��
	RC_Ctl.ch3 = 1024;
	RC_Ctl.switch_left = 0;			//�����Ƹ�
	RC_Ctl.switch_right = 0;		//�����Ƹ�		
	RC_Ctl.mouse.press_left = 0;		//������
	RC_Ctl.mouse.press_right = 0;		//����Ҽ�
	RC_Ctl.mouse.x = 0;	//�����X����ƶ��ٶ�
	RC_Ctl.mouse.y = 0;	//�����Y����ƶ��ٶ�
	RC_Ctl.mouse.z = 0;	//�����Z����ƶ��ٶ�
	RC_Ctl.keyBoard.key_code = 0;
}
/*------���ղ�����ң��������------*/

short int Get_Mode_Data(void)	//����ģʽ����
{
	return RC_Ctl.switch_right;
}


short int Get_ch0_Data(void)	//ͨ�����ݷ���
{
	return RC_Ctl.ch0;
}


short int Get_ch1_Data(void)
{
	return RC_Ctl.ch1;
}


short int Get_ch2_Data(void)
{
	return RC_Ctl.ch2;
}


short int Get_ch3_Data(void)
{
	return RC_Ctl.ch3;
}


short int Get_Mouse_X_Data(void)
{
	return RC_Ctl.mouse.x;
}


short int Get_Mouse_Y_Data(void)
{
	return RC_Ctl.mouse.y;
}





