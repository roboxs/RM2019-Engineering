#ifndef  _DRIVER_DBUS_H
#define	 _DRIVER_DBUS_H

#include<stm32f4xx.h>

/* ----------------------- DJIÒ£¿Ø½á¹¹Ìå ------------------------------------- */
typedef struct
{
		struct
		{
				short ch0;
				short ch1;
				short ch2;
				short ch3;
				uint8_t s1;
				uint8_t s2;
		}rc;
		
		struct
		{
				int16_t x;
				int16_t y;
				int16_t z;
				uint8_t press_l;
				uint8_t press_r;
		}mouse;
		
		struct
		{
				uint16_t v;
		}key;
}RC_Ctl_t;


enum
{
	REMOTEMODE=0,
	MOUSEKEYMODE=1,
	ROTATIONMODE=2,
};



extern volatile unsigned char g_sbus_rx_buffer[];
extern RC_Ctl_t RC_Ctl;


void remote_control(void);
void mousekey_control(void);

#endif


