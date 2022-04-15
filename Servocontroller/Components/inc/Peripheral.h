#ifndef __PERIPHERAL_H
#define __PERIPHERAL_H

#include "main.h"


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

#define TXD2				1<<10 //48
#define RXD2				1<<11 //49
#define INA					1<<9 //76 To INB
#define INB					1<<8 //77 To INA

/**********Port 1**************/
#define INDEX				1<<24 //38
#define PHB					1<<23 //37
#define PHA					1<<20 //34

#define GND_1				1<<1  //94
#define FC_2				1<<8  //92
#define FC_1				1<<10 //90
#define GND_2				1<<15 //88

void NVIC_Config(void);
void Systick_Config(void);
void IO_Config(void);	
void QEI_Config(void);
void USB_Config(void);
void UART_Config(void);
void PWM_Config(void);
void TIM_Config(void);


void UART_Trace(char* format,...);
void UART_Send_Bytes(uint8_t * stream, unsigned int len);
void USB_Disconnect(void);
void USB_Send_Bytes(uint8_t * stream, unsigned int len);
void USB_Trace(char * string, ...);
void Uart_Tx(LPC_UART_TypeDef* uart ,char * format,...);
__ASM void delay(unsigned int d);
void _delay(unsigned int d);
void Delay(uint32_t delay);

#endif


















