#include "main.h"
#include "Dev.h"

Serial Usb;
Button button;
Timer Led_Tmr;
Timer Adc_Tmr;


/******************************* Variable Init **************************/
void USB_Handler_Init(void)
{
	Fifo_Init(&Usb.fifo,10);
	Usb.receiver_EventHandler = (Receiver_EventHandler)USB_Receiver_Handler;
}

void IO_Init(void)
{
	Button_Init(&button,(FuncEdgeHandler)Button_Handler);
	Timer_Init(&Led_Tmr,100);
	Timer_Start(&Led_Tmr);
}

void ADC_Process_Init(void)
{
	Timer_Init(&Adc_Tmr,500);
	Timer_Start(&Adc_Tmr);
}

/********************** Periph Events ************************************/
void ADC_Read_Event(void)
{
	if(Timer_Elapsed(&Adc_Tmr))
	{
		uint32_t adc_value;
		ADC_StartCmd(LPC_ADC,ADC_START_NOW);
		while(!(LPC_ADC->ADSTAT&(uint32_t)(1)));
		adc_value =  ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_0);
		USB_Trace("ADC0 = %d\r\n", adc_value);
		Timer_Start(&Adc_Tmr);
	}
}

void IO_Events(void)
{
	if(Timer_Elapsed(&Led_Tmr))
	{
		Toggle(2,1<<0);
		Timer_Start(&Led_Tmr);
	}
	if(!Read(2,1<<10))
	{
		Button_Fsm(&button,BUTTON_EVT_HIGH);
	}
	else Button_Events(&button);
	
}

/*Check if recieved*/
void USB_Event(void)
{
	Serial_Events(&Usb);
}


