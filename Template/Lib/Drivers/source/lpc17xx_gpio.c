#include "lpc17xx_gpio.h"

void GPIO_Init(char Port, uint32_t Pins, FUNC_e Funcnum, MODE_e PinMode, OD_e Od)
{
	uint8_t PortPin 	 = (uint8_t)Port;
	uint8_t Od_Port    = (uint8_t)Port; /*01234*/
	
	uint8_t PinNum    = (Pins);
	uint8_t Od_PinNum = (Pins);	
		
	PortPin*= 2;
	
	if(PinNum>= 16)
	{
		PortPin ++;
		PinNum -= 16;
	}
	
	PinNum *= 2;
	
	*((uint32_t*)&LPC_PINCON->PINSEL0 + PortPin) &= ~(uint32_t)(0x03 << PinNum); 
	*((uint32_t*)&LPC_PINCON->PINSEL0 + PortPin) |= (uint32_t)(Funcnum << PinNum);
	
	*((uint32_t*)&LPC_PINCON->PINMODE0+ PortPin) &= ~(uint32_t)(0x03 << PinNum); 
  *((uint32_t*)&LPC_PINCON->PINMODE0+ PortPin) |= (uint32_t)(PinMode << PinNum);
	
	if(Od) *((uint32_t*)&LPC_PINCON->PINMODE_OD0 + Od_Port) |= (uint32_t)(1 << Od_PinNum);
	else 	 *((uint32_t*)&LPC_PINCON->PINMODE_OD0 + Od_Port) &= ~(uint32_t)(1 << Od_PinNum);
	
}

void I2C_Pin_Init(char port, char num , char od){

	/*Pending*/
}		












