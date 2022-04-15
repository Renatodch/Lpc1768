/*
 * main.c
 * 	
 *  Created on: 01/07/2018
 *      Author: RENATO
 *  Description: 
 *		- LPC1768 running at 100 MHz
 *		- Cartesian robot controller
 *		- Peripherals : QEI, GPIO, USB, PWM, Systick, NVIC 
 *
 */

#include "main.h"

char Buffer_RAM1[25000] = {0};
//Stack and heap comes after the end of SRAM1. IRAM2 unselected as default
char Buffer_RAM2[32000] __attribute__((at(0x2007C000))) = {0};

/** Velocity Accumulator */
__IO uint64_t VeloAcc;
/** Times of Velocity capture */
__IO uint32_t VeloCapCnt;
/** Flag indicates Times of Velocity capture is enough to read out */
__IO FlagStatus VeloAccFlag;
uint32_t rpm;
uint32_t averageVelo;

Serial Usb;
Button button;
Timer Led_Tmr;
Timer Adc_Tmr;
char init;

int main (void)
{
	/* 5 bits subpriority none preemption*/
	NVIC_SetPriorityGrouping(0x07); 
  Systick_Config();
	IO_Config();
	PWM_Config();
	QEI_Config();
	USB_Config();
	
	for(init = 0 ; init < 50; init ++)
	{
		Toggle(2,1<<0);
		Delay(100);
	}
	
	USB_Trace("*********************************************");
	USB_Trace("LPC1768 Running at 100MHz                  ");
	USB_Trace("Proyecto : %s ",get_version());
	USB_Trace("Fecha de Compilacion : %s ",get_build_date_time());
	USB_Trace("*********************************************");

	
	Fifo_Init(&Usb.fifo,10);
	Usb.receiver_EventHandler = (Receiver_EventHandler)USB_Receiver_Handler;
	
	Button_Init(&button,(FuncEdgeHandler)Button_Handler);
	
	Timer_Init(&Led_Tmr,500);
	Timer_Start(&Led_Tmr);
	
	VeloAccFlag = RESET;
	VeloAcc = VeloCapCnt = 0;
	
	memset(Buffer_RAM2,0xFF,sizeof(Buffer_RAM2));

	while(1)
	{
		Serial_Events(&Usb);
		
		if(Timer_Elapsed(&Led_Tmr))
		{
			//USB_Trace("Posicion actual : %d", LPC_QEI->QEIPOS);
			Toggle(2,1<<0);
			Timer_Start(&Led_Tmr);
		}
		
		if(!Read(2,1<<10))	Button_Fsm(&button,BUTTON_EVT_HIGH);
		
		else Button_Events(&button);
		

		if (VeloAccFlag == SET) {
			// Get Acc
			averageVelo = (uint32_t)(VeloAcc / VeloCapCnt);
			//rpm = QEI_CalculateRPM(LPC_QEI, averageVelo, ENC_RES);
	
			USB_Trace("Frecuencia: %d\r\n",averageVelo);
			
			VeloAccFlag = RESET;
			VeloAcc = 0;
			VeloCapCnt = 0;
		}
	}
	
}



