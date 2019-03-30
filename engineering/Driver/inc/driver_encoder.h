#ifndef _DRIVER_ENCODER_H
#define _DRIVER_ENCODER_H

#include <stm32f4xx.h>




typedef struct
{
	uint16_t ecd;
	uint16_t last_ecd;

	int16_t  speed_rpm;
	int16_t  given_current;
	
	int32_t  round_cnt;
	int32_t  total_ecd;
	int32_t  total_angle;
	float    current_angle;
	
	uint16_t offset_ecd;
	int32_t  ecd_current_value;

} MotoMeasure_t;



void encoder_data_handler(MotoMeasure_t* encoder, CanRxMsg * rcan);


#endif
