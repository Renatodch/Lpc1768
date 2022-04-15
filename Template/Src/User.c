#include "main.h"
#include "user.h"
//Stack and heap comes after the end of SRAM1. IRAM2 unselected as default
//char Buffer_RAM2[4000] __attribute__((at(0x2007C000))) = {0};

extern uint8_t cx[10];
extern uint8_t cy[10];
extern uint8_t cz[10];

void Loop(void)
{
	IO_Events();

//	ADC_Read_Event();
	
	USB_Event();
}


void Variables(void)
{
	USB_Handler_Init();
	
	IO_Init();
	
//	ADC_Process_Init();
}

void Config(void)
{
	Peripheral();
}


