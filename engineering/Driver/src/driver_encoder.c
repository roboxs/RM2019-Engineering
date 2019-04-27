#include <driver_encoder.h>


void encoder_data_handler(MotoMeasure_t * encoder, CanRxMsg * rcan)
{
	encoder->last_ecd = encoder->ecd;
	encoder->ecd      = (uint16_t)((rcan->Data[0]<<8) | rcan->Data[1]);//电调的机械角度
  
	if (encoder->ecd - encoder->last_ecd > 4096)
	{
		encoder->round_cnt--;
	}
	else if (encoder->ecd - encoder->last_ecd < -4096)
	{
		encoder->round_cnt++;
	}
	
	encoder->current_angle	= encoder->ecd * ENCODER_ECD_TO_DEG;//电机角度的当前值
	encoder->total_ecd 		= encoder->round_cnt * 8192 + encoder->ecd;//电机总的编码器值
	encoder->total_angle 	= encoder->total_ecd * ENCODER_ECD_TO_DEG;//电机角度总值
	encoder->speed_rpm     = (int16_t)((rcan->Data[2]<<8) | rcan->Data[3]);
	encoder->given_current = (int16_t)((rcan->Data[4]<<8) | rcan->Data[5]);
}
