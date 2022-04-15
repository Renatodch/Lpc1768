#include "main.h"

extern Trayectoria Eje;

void Button_Handler(void)
{
	//Eje.Ejecutando = 1;
	
	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_TIM_Int, ENABLE);

	Delay(3000000);
	Eje.Prismatica.DC = 8000;  //19% (DC max): Elegimos la supuesta DC minimo para 20 RPM, sin embargo no es asi,porque si probamos
														 //este DC para obtener los 20 rpm obtendremos como 120 rpm como velocidad minima, entonces eso da como 
														 // 2 r/s = 1600 edges/s lo cual nos dice que necesitmaos una Fs mayor, digamos 2KHz == 0.5ms
	
	LPC_PWM1->MR2 = Eje.Prismatica.DC;
	LPC_PWM1->LER |=  (1<<2);
	LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
	LPC_PWM1->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;
	High(0,INB);	// move right
	Low(0,INA);
}

/*Handle tasks*/
void USB_Receiver_Handler(char *buf, int len)
{
	char * p;
	USB_Trace(buf);
	
	if((p=strstr(buf,"RN"))!=NULL)
	{
			Eje.Prismatica.DC = atoi(p+2);
			LPC_PWM1->MR2 = Eje.Prismatica.DC ;
			LPC_PWM1->LER |=  (1<<2);
			LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
			LPC_PWM1->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;
			//Eje.Ejecutando = 1;
			High(0,INB);
			Low(0,INA);
	}
	else if((p=strstr(buf,"T="))!=NULL)  // Activa envio uart 
	{
		if(*(p+2) == 0x31)
			Eje.Ejecutando = 1;
		else
			Eje.Ejecutando = 0;
	}
}










