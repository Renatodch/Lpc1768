#ifndef __PERIPHERAL_H
#define __PERIPHERAL_H

#include "main.h"

#define bit(pin)	(uint32_t)(1<<pin)

#define _CLKPWR_RESET(reg,num,div) 	 	(LPC_SC->reg &=  ~(uint32_t)(div << num))
#define _CLKPWR_SET(reg,num,div)       (LPC_SC->reg |=  (uint32_t)(div << num))

#define _CLKPWR_RESET_PCONP(pos)     	(LPC_SC->PCONP &= ~(uint32_t)(pos))
#define _CLKPWR_SET_PCONP(pos)     	  (LPC_SC->PCONP |= (uint32_t)(pos))

/**** LPC17xx_StdPeriphDrivers *****/
#include "lpc17xx_adc.h"
#include "lpc17xx_can.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_emac.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_i2s.h"
#include "lpc17xx_iap.h"
#include "lpc17xx_mcpwm.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_qei.h"
#include "lpc17xx_rit.h"
#include "lpc17xx_rtc.h"
#include "lpc17xx_spi.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_wdt.h"
/************************************/

/**********Port 2**************/
#define LED 				1<<0
#define INT 				1<<10
/**********Port 0**************/
#define KEY					1<<3
#define TEMP_IN			1<<23


void Peripheral(void);
void USB_Disconnect(void);
void USB_Send_Bytes(uint8_t * stream, unsigned int len);
void USB_Trace(char * string, ...);
void Uart_Tx(LPC_UART_TypeDef* uart ,char * format,...);
__ASM void delay(unsigned int d);
void _delay(unsigned int d);
void Delay(uint32_t delay);

#endif


















