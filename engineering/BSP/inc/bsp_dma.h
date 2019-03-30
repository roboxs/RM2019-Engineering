#ifndef _BSP_DMA_H
#define _BSP_DMA_H

#include <stm32f4xx.h>


extern const u16  RECEIVE_BUF_SIZE 	;//接受缓存区数据的长度
extern const u16  SEND_BUF_SIZE 		;//发送缓存区数据长度
extern u8 g_receive_buff[];//接受缓存区
extern u8 g_send_buff[]   ;

void BSP_DMA_Init(void);

#endif

