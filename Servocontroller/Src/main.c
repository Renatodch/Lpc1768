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

extern __IO uint32_t DelayC;

/** Velocity Accumulator */
__IO uint64_t VeloAcc;
__IO uint32_t VeloCapCnt;
__IO FlagStatus VeloAccFlag;

uint32_t rpm;
uint32_t averageVelo;

Serial Usb;

Button button;
Timer Led_Tmr;

Trayectoria Eje;

char init;

void Led_Tick(void)
{
	if(Timer_Elapsed(&Led_Tmr))
	{
		Toggle(2,1<<0);
		Timer_Start(&Led_Tmr);
	}
}

int main (void)
{
	/* 5 bits sub none pre*/
	NVIC_SetPriorityGrouping(0x07); 
  Systick_Config();
	IO_Config();
	PWM_Config();
	QEI_Config();
	TIM_Config();
	USB_Config();
	UART_Config();

	for(init = 0 ; init < 50; init ++)
	{
		Toggle(2,1<<0);
		Delay(100000);
	}

	USB_Trace("*********************************************");
	USB_Trace("LPC1768 Running at 100MHz                    ");
	USB_Trace("Project : %s ",get_version());
	USB_Trace("Date : %s ",get_build_date_time());
	USB_Trace("*********************************************");

	Fifo_Init(&Usb.fifo,10000);
	Usb.receiver_EventHandler = (Receiver_EventHandler)USB_Receiver_Handler;
	
	Button_Init(&button,(FuncEdgeHandler)Button_Handler);
	
	Timer_Init(&Led_Tmr,500000);
	Timer_Start(&Led_Tmr);

	VeloAccFlag = RESET;
	VeloAcc = VeloCapCnt = 0;
	
	Eje.Ejecutando = 0;
	Eje.Prismatica.Dir = NONE;
	
	Pos_Init();
		
	while(1)
	{
		//DelayC = 1000;	

		Serial_Events(&Usb);
		
		Led_Tick();
				
		if(!Read(2,1<<10))	Button_Fsm(&button,BUTTON_EVT_HIGH);
		else Button_Events(&button);
		
		if(!Read(1,FC_1)&&(Eje.Prismatica.Dir != LEFT))
		{
			Eje.Ejecutando = 0;
			Eje.Prismatica.Dir = LEFT;
			High(0,INB|INA);
		}
		else if(!Read(1,FC_2)&&(Eje.Prismatica.Dir != RIGHT))
		{
			Eje.Ejecutando = 0;
			Eje.Prismatica.Dir = RIGHT;
			High(0,INB|INA);
		}
		
		
		if((LPC_QEI->QEIPOS > 1150000) && (LPC_QEI->QEIPOS <= 1200000))
			Eje.Edges = 0;
		else		
			Eje.Edges = LPC_QEI->QEIPOS;	
		
		//while(DelayC!=0);
	}
	
}



