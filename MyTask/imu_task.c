#include "imu_task.h"
#include "inv_mpu.h"
#include "mpu9250.h"
#include "main.h"
#include "cmsis_os.h"

IMU_t IMU;

void Get_IMU_data()
{	
	if((mpu_dmp_get_data(&IMU.pitch,&IMU.roll,&IMU.yaw))==0){
//		MPU_Get_Accelerometer(&IMU.aacx,&IMU.aacy,&IMU.aacz);	//得到加速度传感器数据
//		MPU_Get_Gyroscope(&IMU.gyrox,&IMU.gyroy,&IMU.gyroz);	//得到陀螺仪数据
//		IMU.aacx/=164,IMU.aacy/=164,IMU.aacz/=164;
//		IMU.gyrox/=16.4,IMU.gyroy/=16.4,IMU.gyroz/=16.4;
		HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_SET);
	  osDelay(10);}
}


