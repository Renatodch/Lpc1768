/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERIAL_H
#define __SERIAL_H

#ifdef __cplusplus
 extern "C" 
{
#endif
	

#include "lpc17xx.h"	
#include "Fifo.h"
#include "GPIO.h"

typedef void (*Receiver_EventHandler)(char* buf , int len);
	
typedef struct
{
		Fifo fifo;
		Receiver_EventHandler receiver_EventHandler;
		//USART_TypeDef *Port;
		InOut Rx_Led;
}Serial;

typedef struct
{
		Fifo fifo;
		Receiver_EventHandler receiver_EventHandler;
		
}USB;


void Serial_Init(Serial *p, Receiver_EventHandler receiver_EventHandler);
void Usb_InitVar(Serial *p, Receiver_EventHandler receiver_EventHandler);
void Serial_Send(Serial *p, char *msg);
void Serial_Timer(Serial *p);
void Serial_Events(Serial *p);

#ifdef __cplusplus
}
#endif

#endif 

