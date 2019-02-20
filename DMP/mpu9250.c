#include "mpu9250.h"
#include "stm32f4xx_hal_i2c.h"
#include "i2c.h"


//��ʼ��MPU9250 �ɲ��ã���������
//����ֵ:0,�ɹ�
//    ����,�������
//unsigned char MPU9250_Init(void)
//{
//	HAL_StatusTypeDef status;
//	HAL_I2C_StateTypeDef flag;
//	unsigned char pdata;
//	//����豸�Ƿ�׼����  ��ַ   ��� ���� ��ʱʱ��  
//	status=HAL_I2C_IsDeviceReady(&hi2c1,MPU9250_ADDR, 10, HAL_MAX_DELAY);
//    unsigned char res=0;
//////    IIC_Init();     //��ʼ��IIC����
//	pdata=0x80; //��λMPU
//	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY); //д0x80��λ
////   MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
//    HAL_Delay(500);  //��ʱ100ms
//	//����MPU
//	pdata=0x01; // 7λ 1 ����  6λ˯��ģʽ1 ˯�� 2 ����   3λ Ϊu�㶮������0 ����    0-2λ ʱ��ѡ��  01 PLLʹ��XZ��������Ϊ����
//	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY); //д0x80��λ
////    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
//	//�������� �Ĵ���  2000  3
//	pdata=3<<3; //����3 Ϊ����2000  ����3λ ����Ӧ�� 3 4 λ�Ĵ�����
//	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_GYRO_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
////    MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps
//	//���ü��ٶȴ��������� 2g
//	pdata=0; 
//	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_ACCEL_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
////	  MPU_Set_Accel_Fsr(0);					       	 	//���ٶȴ�����,��2g
////  pdata=50; //1000/50-1  �������Ҫ�����ϲ鿴  ԭ�� �ͼ��㷽��
////	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_SAMPLE_RATE_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//    MPU_Set_Rate(50);						       	 	//���ò�����50Hz
//  pdata=0;
//	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_INT_EN_REG, 1, &pdata, 1, HAL_MAX_DELAY);
////    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
//  pdata=0;
//	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_USER_CTRL_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
////	  MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
//  pdata=0;
//	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_FIFO_EN_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
////	  MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
//  pdata=0x82;
//	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_INTBP_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
////	  MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
//  pdata=MPU_DEVICE_ID_REG;
//  res=HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR, MPU_DEVICE_ID_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
////    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //��ȡMPU6500��ID
//    if(res==MPU6500_ID) //����ID��ȷ
//    {
//			pdata=0x01;
//			HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
////        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
//			pdata=0x00;
//			HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MPU_PWR_MGMT2_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
////        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
////			pdata=50; //1000/50-1  �������Ҫ�����ϲ鿴  ԭ�� �ͼ��㷽��
////	    HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_SAMPLE_RATE_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//		    MPU_Set_Rate(50);						       	//���ò�����Ϊ50Hz   
//    }else return 1;
//  pdata=MAG_WIA;
//  res=HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,MAG_WIA, 1, &pdata, 1, HAL_MAX_DELAY); 
////    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//��ȡAK8963 ID   
//    if(res==AK8963_ID)
//    {
//			pdata=0x11;
//			HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MAG_CNTL1, 1, &pdata, 1, HAL_MAX_DELAY); 
////        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//����AK8963Ϊ���β���ģʽ
//    }else return 1;

//    return 0;
//}
//����MPU9250�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
unsigned char MPU_Set_LPF(u16 lpf)
{
	unsigned char data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
  return HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_CFG_REG, 1, &data, 1, HAL_MAX_DELAY); 	//�������ֵ�ͨ�˲���  
}

//����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
unsigned char MPU_Set_Rate(u16 rate)
{
	unsigned char data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MPU_SAMPLE_RATE_REG, 1, &data, 1, HAL_MAX_DELAY);//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}
//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
	unsigned char buf[2]; 
	float temp;
	short raw;
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, MPU_TEMP_OUTH_REG, 1, buf, 2, HAL_MAX_DELAY); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
unsigned char MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
	HAL_StatusTypeDef status;
    unsigned char buf[6]; 
	status=HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,MPU_GYRO_XOUTH_REG, 1,buf, 6, HAL_MAX_DELAY); 
	if(status==HAL_OK)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return status;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
unsigned char MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
	HAL_StatusTypeDef status;
    unsigned char buf[6]={0}; 
	status=HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 1,buf, 6, HAL_MAX_DELAY);
	if(status==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return status;
}

//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
unsigned char MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
	HAL_StatusTypeDef status;
	unsigned char pdata=0X11;
    unsigned char buf[6];
	status=HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR,MAG_XOUT_L, 1,buf, 6, HAL_MAX_DELAY);
	if(status==0)
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	} 	
		HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR,MAG_CNTL1, 1, &pdata, 1, HAL_MAX_DELAY); 
    return status;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
	HAL_I2C_Mem_Write(&hi2c1, ((addr<<1)|0), reg, 1, (unsigned char *)buf, len, HAL_MAX_DELAY);
	return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{ 
	HAL_I2C_Mem_Read(&hi2c1, ((addr<<1)|1), reg, 1, (unsigned char *)buf, len, HAL_MAX_DELAY);
	return 0;
	
}


