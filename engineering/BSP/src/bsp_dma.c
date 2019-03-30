#include <bsp_dma.h>
#include <driver_dbus.h>

const u16  RECEIVE_BUF_SIZE = 1000	;//接受缓存区数据的长度
const u16  SEND_BUF_SIZE 		= 1000	;//发送缓存区数据长度

u8 g_receive_buff[RECEIVE_BUF_SIZE]={0};//接受缓存区
u8 g_send_buff[SEND_BUF_SIZE]={0};



void BSP_DMA_Init(void)
{
	DMA_InitTypeDef DMA_Structure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	
											/*USART6->DMA发送配置*/
	DMA_DeInit(DMA2_Stream7);
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//等待DMA可配置 
	
	DMA_Structure.DMA_Channel 					= DMA_Channel_5;  //通道选择
	DMA_Structure.DMA_PeripheralBaseAddr 		= (u32)&USART6->DR;//DMA外设地址
	DMA_Structure.DMA_Memory0BaseAddr 			= (u32)g_send_buff;//DMA 存储器0地址
	DMA_Structure.DMA_DIR 						= DMA_DIR_MemoryToPeripheral;//存储器到外设模式
	DMA_Structure.DMA_BufferSize 				= SEND_BUF_SIZE;//数据传输量 
	DMA_Structure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_Structure.DMA_MemoryInc 				= DMA_MemoryInc_Enable;//存储器增量模式
	DMA_Structure.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_Structure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMA_Structure.DMA_Mode 						= DMA_Mode_Normal;// 使用普通模式 
	DMA_Structure.DMA_Priority 					= DMA_Priority_Medium;//中等优先级
	DMA_Structure.DMA_FIFOMode 					= DMA_FIFOMode_Disable;         
	DMA_Structure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
	DMA_Structure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_Structure.DMA_PeripheralBurst 			= DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream7, &DMA_Structure);
	DMA_Cmd(DMA2_Stream7,ENABLE);	
		
											/*USART6->DMA接收配置*/
	DMA_DeInit(DMA2_Stream2);
	while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}//等待DMA可配置 
		
	DMA_Structure.DMA_BufferSize				= RECEIVE_BUF_SIZE;
	DMA_Structure.DMA_Channel					= DMA_Channel_5;
	DMA_Structure.DMA_DIR						= DMA_DIR_PeripheralToMemory;//外设到存储单元
	DMA_Structure.DMA_FIFOMode					= DMA_FIFOMode_Disable;
	DMA_Structure.DMA_FIFOThreshold				= DMA_FIFOThreshold_Full;
	DMA_Structure.DMA_Memory0BaseAddr			= (u32)g_receive_buff;
	DMA_Structure.DMA_MemoryBurst				= DMA_MemoryBurst_Single;//储存单元突发单次传输
	DMA_Structure.DMA_MemoryDataSize			= DMA_MemoryDataSize_Byte;//8位
	DMA_Structure.DMA_MemoryInc					= DMA_MemoryInc_Enable;//存储单元增量开启
	DMA_Structure.DMA_Mode						= DMA_Mode_Normal;
	DMA_Structure.DMA_PeripheralBaseAddr		= (u32)&USART6->DR;
	DMA_Structure.DMA_PeripheralBurst			= DMA_PeripheralBurst_Single;
	DMA_Structure.DMA_PeripheralDataSize		= DMA_PeripheralDataSize_Byte;//8位
	DMA_Structure.DMA_PeripheralInc				= DMA_PeripheralInc_Disable;//外设地址增量关闭
	DMA_Structure.DMA_Priority					= DMA_Priority_Medium;//DMA中断优先级
	DMA_Init(DMA2_Stream2,&DMA_Structure);//使用DMA2的数据流2 和USART6_RX相连
	DMA_Cmd(DMA2_Stream2,ENABLE);	
	
		
											/*DBUS->DMA接收配置*/
	DMA_DeInit(DMA2_Stream5);
	DMA_Structure.DMA_Channel 					= DMA_Channel_4;
	DMA_Structure.DMA_PeripheralBaseAddr 		= (uint32_t)&(USART1->DR);
	DMA_Structure.DMA_Memory0BaseAddr 			= (uint32_t)g_sbus_rx_buffer;
	DMA_Structure.DMA_DIR 						= DMA_DIR_PeripheralToMemory;
	DMA_Structure.DMA_BufferSize 				= 18;
	DMA_Structure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
	DMA_Structure.DMA_MemoryInc 				= DMA_MemoryInc_Enable;
	DMA_Structure.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
	DMA_Structure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
	DMA_Structure.DMA_Mode 						= DMA_Mode_Circular;//DMA循环模式
	DMA_Structure.DMA_Priority 					= DMA_Priority_VeryHigh;
	DMA_Structure.DMA_FIFOMode 					= DMA_FIFOMode_Disable;
	DMA_Structure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_1QuarterFull;
	DMA_Structure.DMA_MemoryBurst 				= DMA_Mode_Normal;
	DMA_Structure.DMA_PeripheralBurst 			= DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5,&DMA_Structure);
	DMA_Cmd(DMA2_Stream5,ENABLE);
	
}
