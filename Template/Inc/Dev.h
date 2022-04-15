#ifndef __DEV_H
#define __DEV_H



void USB_Handler_Init(void);
void ADC_Process_Init(void);
void IO_Init(void);

void ADC_Read_Event(void);
void IO_Events(void);
void USB_Event(void);
#endif


