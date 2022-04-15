#ifndef __HANDLERS_H
#define __HANDLERS_H

#include "stdint.h"

void Button_Handler(void);

void USB_Receiver_Handler(char *buf, int len);


typedef __packed struct  /*Use __packed to make data be aligned 1 byte boundary*/
{
	char uno;
	char dos;
	char tres;
	char cuatro;
	uint32_t cinco;
	char seis;
	uint16_t siete;
	uint32_t ocho;
	char boundarie;
}Excelente;


#endif
