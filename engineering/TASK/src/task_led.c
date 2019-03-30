#include <task_led.h>
#include <delay.h>



void led1_task(void *pvParameters)
{
	u32 last_wake_time = xTaskGetTickCount();
	xTaskCreate(led2_task, "led2_task", 	256, 	 NULL, 1, NULL);
	  while(1)
    {
		LED8=!LED8;
        vTaskDelay(500/portTICK_RATE_MS);
		
    }
}

void led2_task(void *pvParameters)
{
	u32 last_wake_time = xTaskGetTickCount();
	while(1)
	{
        LED3=!LED3;
		vTaskDelayUntil(&last_wake_time , 1000/portTICK_RATE_MS);
	}
	//vTaskDelete(NULL);
	
}
