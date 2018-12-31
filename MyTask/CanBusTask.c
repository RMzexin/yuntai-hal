#include "can.h"
#include "CanBusTask.h"
#include "RemoteTask.h"	
#include "PIDcontrolTask.h"
#include "CanBusTask.h"
#include "DataProcessingTask.h"
#include <math.h>

static uint32_t can_count = 0;

volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPluckEncoder = {0,0,0,0,0,0,0,0,0};


/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;
	static CanTxMsgTypeDef		Tx1Message;
	static CanRxMsgTypeDef 		Rx1Message;


	CAN_FilterConfigStructure.FilterNumber = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.BankNumber = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		//err_deadloop(); //show error!
	}


	if(_hcan == &hcan1){
		_hcan->pTxMsg = &Tx1Message;
		_hcan->pRxMsg = &Rx1Message;
	}


}

uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
   can_count++;
	if(HAL_GetTick() - FlashTimer>500){
		FlashTimer = HAL_GetTick();	
	}
		static Bool first = true;
	if(can_count<=5&&first){
		Chassis_And_Gimbal_Data_Init();}
	if(can_count==5&&first){
		first = false ;}

	switch(_hcan->pRxMsg->StdId)
		{
		case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM1Encoder ,_hcan):EncoderProcess(&CM1Encoder ,_hcan);       //获取到编码器的初始偏差值            
			}break;
		case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM2Encoder ,_hcan):EncoderProcess(&CM2Encoder ,_hcan);
			}break;
		case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM3Encoder ,_hcan):EncoderProcess(&CM3Encoder ,_hcan);   
			}break;
		case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
			{
				(can_count<=50) ? GetEncoderBias(&CM4Encoder ,_hcan):EncoderProcess(&CM4Encoder ,_hcan);
			}break;
		case CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID:
			{
			  EncoderProcess(&GMYawEncoder ,_hcan);
        GMYawEncoder.ecd_angle = Yaw_Angle_Precision_Filter(&GMYawEncoder);					
			}break;
		case CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID:
			{
				EncoderProcess(&GMPitchEncoder ,_hcan);
			  GMPitchEncoder.ecd_angle = Pitch_Angle_Precision_Filter(&GMPitchEncoder);
				//码盘中间值设定也需要修改
				if(can_count<=100)
					{
						if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) <-4000)
							{
								GMPitchEncoder.ecd_bias =0 + 8192;
							}
							else if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 4000)
								{
									GMPitchEncoder.ecd_bias =0 - 8192;
								}
					}
			}break;	
		case CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID:
			{
				EncoderProcess(&GMPluckEncoder ,_hcan);
				GMPluckEncoder.ecd_angle = Pluck_Angle_Precision_Filter(&GMPluckEncoder);				
			}break;				
		}
/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
}

void GetEncoderBias(volatile Encoder *v, CAN_HandleTypeDef*_hcan)
{

            v->ecd_bias = (_hcan->pRxMsg->Data[0]<<8)|_hcan->pRxMsg->Data[1];  //保存初始编码器值作为偏差  
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
}
/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/
void EncoderProcess(volatile Encoder *v,CAN_HandleTypeDef* _hcan)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value =(_hcan->pRxMsg->Data[0]<<8)|_hcan->pRxMsg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>7500)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);	
  if(_hcan->pRxMsg->StdId>=0x201||_hcan->pRxMsg->StdId<=0x204)
	{
		if(_hcan->pRxMsg->Data[2] & 128)
			{		
			v->rotor_speed =(~(65535-((_hcan->pRxMsg->Data[2]<<8)|_hcan->pRxMsg->Data[3]))/19);
			}
			else
			{
				v->rotor_speed =(((_hcan->pRxMsg->Data[2]<<8)|_hcan->pRxMsg->Data[3])/19);				
			}
	}		
}

float Yaw_Angle_Precision_Filter(volatile Encoder *Y)	//精准值   +-0.05°
{
	static Bool first = true;
	static float yaw_angle_temp = 0;
	
	if(first == true)
	{
		first = false;
		yaw_angle_temp = Y->ecd_angle;
		
		return yaw_angle_temp;
	}
	 
	if( fabs(Y->ecd_angle - yaw_angle_temp) <= 0.05 )
	{
		return yaw_angle_temp;
	}
	else 
	{
		yaw_angle_temp = Y->ecd_angle;
		
		return Y->ecd_angle;
	}
}

float Pluck_Angle_Precision_Filter(volatile Encoder *L)	//精准值   +-0.05°
{
	static Bool first = true;
	static float pluck_angle_temp = 0;
	
	if(first == true)
	{
		first = false;
		pluck_angle_temp = L->ecd_angle;
		
		return pluck_angle_temp;
	}
	 
	if( fabs(L->ecd_angle - pluck_angle_temp) <= 0.05 )
	{
		return pluck_angle_temp;
	}
	else 
	{
		pluck_angle_temp = L->ecd_angle;
		
		return L->ecd_angle;
	}
}

float Pitch_Angle_Precision_Filter(volatile Encoder *P)
{
	static Bool first = true;
	static float pitch_angle_temp = 0;
	
	if(first == true)
	{
		first = false;
		pitch_angle_temp = P->ecd_angle;
		
		return pitch_angle_temp;
	}
	
	if( fabs(P->ecd_angle - pitch_angle_temp) <= 0.05)
	{
		return pitch_angle_temp;
	}
	else 
	{
		pitch_angle_temp = P->ecd_angle;
		
		return P->ecd_angle;
	}
}


/********************************************************************************
   给底盘电调板发送指令，ID号为0x200８底盘返回ID为0x201-0x204
*********************************************************************************/
void Set_CM_Speed(CAN_HandleTypeDef* hcan, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq){

	hcan->pTxMsg->StdId = 0x200;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = (cm1_iq >> 8);
	hcan->pTxMsg->Data[1] =  cm1_iq;
	hcan->pTxMsg->Data[2] = (cm2_iq >> 8);
	hcan->pTxMsg->Data[3] =  cm2_iq;
	hcan->pTxMsg->Data[4] = (cm3_iq >> 8);
	hcan->pTxMsg->Data[5] =  cm3_iq;
	hcan->pTxMsg->Data[6] = (cm4_iq >> 8);
	hcan->pTxMsg->Data[7] =  cm4_iq;
	
	HAL_CAN_Transmit(hcan, 100);
}
/********************************************************************************
   给电调板发送指令，ID号为0x1FF，用三个个电调板，数据回传ID为0x205和0x206和0x207
	 cyq:更改为发送三个电调的指令。
*********************************************************************************/
void Set_Gimbal_Current(CAN_HandleTypeDef* hcan, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_pluck_iq){
	
	hcan->pTxMsg->StdId = 0x1FF;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = (gimbal_yaw_iq >> 8);
	hcan->pTxMsg->Data[1] =  gimbal_yaw_iq;
	hcan->pTxMsg->Data[2] = (gimbal_pitch_iq >> 8);
	hcan->pTxMsg->Data[3] =  gimbal_pitch_iq;
	hcan->pTxMsg->Data[4] = (gimbal_pluck_iq >> 8);
	hcan->pTxMsg->Data[5] =  gimbal_pluck_iq;
	
	HAL_CAN_Transmit(hcan, 100);
}
