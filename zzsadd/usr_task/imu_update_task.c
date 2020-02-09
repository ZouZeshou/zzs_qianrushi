#include "imu_update_task.h"
#include "drv_imu.h"
void StartTask05(void const * argument)
{
	portTickType task_start_time;
  for(;;)
  {
		task_start_time = xTaskGetTickCount();
		taskENTER_CRITICAL();
		mpu_get_data(&sensor);
		UpdateIMU(&sensor);
		//mahony_ahrs_updateIMU(&sensor,&atti);
		imu_temp_keep();
		taskEXIT_CRITICAL();
    osDelayUntil(&task_start_time,2);
  }

}
