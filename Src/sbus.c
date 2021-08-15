#include "SBUS.h"
#include "main.h"

SBUS_Buffer SBUS;
uint8_t First_Byte_flag_SBUS=1; //首字节标志
uint8_t SBUS_RX_Finish=0;
uint8_t SBUS_RXIndex = 0;//当前接收字节数
uint8_t SBUS_RXBuffer[SBUS_RX_LEN] = {0};//接收缓冲
uint8_t end_flge[2] = {0xaa,0xbb};
uint8_t error[1]={0xcc};
uint8_t RX_databuf[26]={0};

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern uint8_t Uart3_receive_buf[50];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
   LED_Red_Toggle();
   if(Uart3_receive_buf[0] == 0x0f) 
   {
     HAL_UART_Transmit(&huart1, Uart3_receive_buf, 25, 10);  
     HAL_UART_Transmit(&huart1, end_flge, 2, 10);
   }
   else
     HAL_UART_Transmit(&huart1, error, 1, 10);
}




void SBUS_Handle()
{
	if(SBUS_RX_Finish==1)
	{
		SBUS_RX_Finish=0;//准备下一次接收
		NVIC_DisableIRQ(USART3_IRQn);//从UART_RXBuffer读取数据过程中要关闭中断，防止读写混乱
		SBUS.Start=SBUS_RXBuffer[0];
		SBUS.Ch1=((uint16_t)SBUS_RXBuffer[1])|((uint16_t)((SBUS_RXBuffer[2]&0x07)<<8));
		SBUS.Ch2=((uint16_t)((SBUS_RXBuffer[2]&0xf8)>>3))|(((uint16_t)(SBUS_RXBuffer[3]&0x3f))<<6);
		SBUS.Ch3=((uint16_t)((SBUS_RXBuffer[3]&0xc0)>>6))|((((uint16_t)SBUS_RXBuffer[4])<<2))|(((uint16_t)(SBUS_RXBuffer[5]&0x01))<<10);
		SBUS.Ch4=((uint16_t)((SBUS_RXBuffer[5]&0xfe)>>1))|(((uint16_t)(SBUS_RXBuffer[6]&0x0f))<<7);
		SBUS.Ch5=((uint16_t)((SBUS_RXBuffer[6]&0xf0)>>4))|(((uint16_t)(SBUS_RXBuffer[7]&0x7f))<<4);
		SBUS.Ch6=((uint16_t)((SBUS_RXBuffer[7]&0x80)>>7))|(((uint16_t)SBUS_RXBuffer[8])<<1)|(((uint16_t)(SBUS_RXBuffer[9]&0x03))<<9);
		SBUS.Ch7=((uint16_t)((SBUS_RXBuffer[9]&0xfc)>>2))|(((uint16_t)(SBUS_RXBuffer[10]&0x1f))<<6);
		SBUS.Ch8=((uint16_t)((SBUS_RXBuffer[10]&0xe0)>>5))|(((uint16_t)(SBUS_RXBuffer[11]))<<3);
		SBUS.Ch9=((uint16_t)SBUS_RXBuffer[12])|(((uint16_t)(SBUS_RXBuffer[13]&0x07))<<8);
		SBUS.Ch10=((uint16_t)((SBUS_RXBuffer[13]&0xf8)>>3))|(((uint16_t)(SBUS_RXBuffer[14]&0x3f))<<5);
		SBUS.Ch11=((uint16_t)((SBUS_RXBuffer[14]&0xc0)>>6))|(((uint16_t)SBUS_RXBuffer[15])<<2)|(((uint16_t)(SBUS_RXBuffer[16]&0x01))<<10);
		SBUS.Ch12=((uint16_t)((SBUS_RXBuffer[16]&0xfe)>>1))|(((uint16_t)(SBUS_RXBuffer[17]&0x0f))<<7);
		SBUS.Ch13=((uint16_t)((SBUS_RXBuffer[17]&0xf0)>>4))|(((uint16_t)(SBUS_RXBuffer[18]&0x7f))<<4);
		SBUS.Ch14=((uint16_t)((SBUS_RXBuffer[18]&0x80)>>7))|(((uint16_t)SBUS_RXBuffer[19])<<1)|(((uint16_t)(SBUS_RXBuffer[20]&0x03))<<9);
		SBUS.Ch15=((uint16_t)((SBUS_RXBuffer[20]&0xfc)>>2))|(((uint16_t)(SBUS_RXBuffer[21]&0x1f))<<6);
		SBUS.Ch16=((uint16_t)((SBUS_RXBuffer[21]&0xe0)>>5))|(((uint16_t)SBUS_RXBuffer[22])<<3);
		SBUS.Flag=SBUS_RXBuffer[23];
	    SBUS.End=SBUS_RXBuffer[24];
		NVIC_EnableIRQ(USART3_IRQn);
	}
}
