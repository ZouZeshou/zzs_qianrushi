#include "imu_update_task.h"
#include "drv_imu.h"
#include "global.h"
void StartTask05(void const * argument)
{
	portTickType task_start_time;
  for(;;)
  {
		task_start_time = xTaskGetTickCount();
		taskENTER_CRITICAL();
		mpu_get_data(&sensor);
		if(USE_MAHONY_METHOD)
			mahony_ahrs_updateIMU(&sensor,&atti);
		else
			UpdateIMU(&sensor);
		imu_temp_keep();
		taskEXIT_CRITICAL();
    osDelayUntil(&task_start_time,2);
  }
}
