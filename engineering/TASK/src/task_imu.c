#include <task_imu.h>
#include <task_led.h>
#include <driver_imu.h>
#include <inv_mpu.h>


Gyroscope_s cloud_gyroscope;
Gyroscope_s chassis_gyroscope;

void imu_task(void *pvParameters)
{
	float pitch, roll, yaw;
	short gx,gy,gz;
	while(1)
	{
		LED2=~LED2;
		mpu_dmp_get_data(&pitch, &roll, &yaw);
		MPU_Get_Gyroscope(&gx, &gy, &gz);
		
		cloud_gyroscope.pitch = pitch;
		cloud_gyroscope.roll  = roll;
		cloud_gyroscope.yaw   = yaw;
		cloud_gyroscope.gx = -gx;
		cloud_gyroscope.gy = -gy;
		cloud_gyroscope.gz = -gz;
		
		chassis_gyroscope.pitch = pitch;
		chassis_gyroscope.roll  = roll;
		chassis_gyroscope.yaw   = yaw;
		chassis_gyroscope.gx = gx;
		chassis_gyroscope.gy = gy;
		chassis_gyroscope.gz = gz;
		vTaskDelay(5);
	}
}

