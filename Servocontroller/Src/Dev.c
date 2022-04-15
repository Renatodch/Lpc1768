#include "main.h"
#include "Dev.h"
extern Trayectoria Eje;

void Pos_Init(void)
{
	if(Read(1,FC_1))
	{
		LPC_PWM1->MR2 = 50000;
		LPC_PWM1->LER |=  (1<<2);
		LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
		LPC_PWM1->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;
		Low(0,INB);
		High(0,INA);	
	}
	while(Read(1,FC_1));
	Eje.Prismatica.Dir = LEFT;

	High(0,INB|INA);
	
}

/********************** Periph Events ************************************/
void ADC_Read_Event(void)
{
//	if(Timer_Elapsed(&Adc_Tmr))
//	{
//		uint32_t adc_value;
//		ADC_StartCmd(LPC_ADC,ADC_START_NOW);
//		while(!(LPC_ADC->ADSTAT&(uint32_t)(1)));
//		adc_value =  ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_0);
//		USB_Trace("ADC0 = %d\r\n", adc_value);
//		Timer_Start(&Adc_Tmr);
//	}
}

//void IO_Events(void)
//{
//	if(Timer_Elapsed(&Led_Tmr))
//	{
//		Toggle(2,1<<0);
//		Timer_Start(&Led_Tmr);
//	}
//	if(!Read(2,1<<10))
//	{
//		Button_Fsm(&button,BUTTON_EVT_HIGH);
//	}
//	else Button_Events(&button);
//	
//}


void QEI_Event(void)
{

}

