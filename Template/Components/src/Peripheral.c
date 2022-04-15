#include "main.h"
#include "usb.h"
#include "usbhw.h"
#include "usbcfg.h"
#include "usbcore.h"
#include "cdc.h"

extern __IO char All_Packets_Sent;

__IO uint32_t DelayC;

char UART_Buffer[1000]={0};
char USB_Buffer[1000]={0};


void IWDT_Config(void)
{
	uint32_t TimeOut = 5000000; /*in uS*/
	WDT_Init(WDT_CLKSRC_PCLK, WDT_MODE_RESET, TimeOut);
}

void ADC_Config(void)
{	
	CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCAD, ENABLE);
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_ADC,CLKPWR_PCLKSEL_CCLK_DIV_4);

//	/*Sensor de Temperatura en LPC board*/
//	Input(0,1<<23);
//	GPIO_Init(0, 23, GPIO_FUNC_1, GPIO_MODE_FLOATING ,GPIO_NO_OD);
	
//	ADC_Init(LPC_ADC,200000);
//	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_0,ENABLE);
	
	
	
	/*Potenciómetro*/
	Input(0,1<<3); 
	GPIO_Init(0, 3, GPIO_FUNC_2, GPIO_MODE_FLOATING ,GPIO_NO_OD);
	
	ADC_Init(LPC_ADC,200000);
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_6,ENABLE);
	
	/*Sharp*/
	Input(0,1<<2); 
	GPIO_Init(0, 2, GPIO_FUNC_2, GPIO_MODE_FLOATING ,GPIO_NO_OD);
	
	ADC_Init(LPC_ADC,200000);
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_7,ENABLE);
}

void IO_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	Turn_On(CLKPWR_PCONP_PCGPIO);
	
	/******* Outputs ********/
	Output(2,1<<0);
	High(2,1<<0);
	GPIO_Init(2,0,GPIO_DEFAULT,GPIO_MODE_PU,GPIO_NO_OD);
	
	/****** Inputs **********/
	Input(2,1<<10);
	Set_edge(2,1<<10,F);
	Clr_int(2,1<<10);
		
	GPIO_Init(2,10,GPIO_DEFAULT,GPIO_MODE_PU,GPIO_NO_OD);
	NVIC_InitStructure.IRQHandler = EINT3_IRQn;      /*Port 0-2 As basis, works with EINT3*/
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_InitStructure.SubPriority = 5;
	NVIC_Init(&NVIC_InitStructure);

}


void TIM_Config(void)
{
	//TIM_CAPTURE_InitTypeDef CaptureStructure;
	TIM_MATCH_InitTypeDef MatchStructure;
	TIM_TIMER_InitTypeDef TimerStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM0,ENABLE);
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0,CLKPWR_PCLKSEL_CCLK_DIV_4);
	/*TC increments every match of PR (PCLK = 100MHz/4) if PR = 0 TC every PCLK = 25MHz*/
	TimerStructure.PrescaleValue = 1;	
	TimerStructure.CounterOption = TIM_MODE_TIMER;
	TimerStructure.CountInputSelect = TIM_COUNTER_INCAP0;
	
	MatchStructure.IntOnMatch = ENABLE;
	MatchStructure.StopOnMatch = DISABLE;
	MatchStructure.ResetOnMatch = ENABLE;
	MatchStructure.MatchChannel = 0;  
	MatchStructure.MatchValue = 25000000; /*prescaler = 1  is 0 then T = 1 s*/
	MatchStructure.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	
	TIM_Init(LPC_TIM0,&TimerStructure);
	TIM_ConfigMatch(LPC_TIM0,&MatchStructure);
	
	TIM_Cmd(LPC_TIM0,ENABLE);
	
	NVIC_InitStructure.IRQHandler = TIMER0_IRQn;
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.SubPriority = 0;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

extern uint32_t MR0;
void PWM_Config(void)
{
	PWM_TIMERCFG_Type PWMCfgDat;
	PWM_MATCHCFG_Type PWMMatchCfgDat;
	NVIC_InitTypeDef NVIC_InitStructure;

	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCPWM1,ENABLE);
	CLKPWR_SetPCLKDiv (CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_1);
	
	/*Pin Config*/
	Output(2,1<<1|1<<2|1<<3);

	GPIO_Init(2,1,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD); //channel 2 pin 74
	GPIO_Init(2,2,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD); //channel 3 pin 73
	GPIO_Init(2,3,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD); //channel 4 pin 70

	
	PWMCfgDat.PrescaleValue = 1;
	PWM_Init(LPC_PWM1, PWM_MODE_TIMER, (void *) &PWMCfgDat); //PCLK = 25 MHz
	
	/* Set match value for PWM match channel, value to reach by the clock. If 25M then T = 1 s*/
	//T = PCLK * MatchValue * Pre 
	MR0 = 1000000;
	PWM_MatchUpdate(LPC_PWM1, 0, 1000000, PWM_MATCH_UPDATE_NOW);
	/* PWM Timer/Counter will be reset when channel 0 matching
	 * Enable interrupt when match
	 * no stop when match */
	PWMMatchCfgDat.IntOnMatch = ENABLE;
	PWMMatchCfgDat.MatchChannel = 0;
	PWMMatchCfgDat.ResetOnMatch = ENABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);
		 
	/* Configure PWM channel edge option
	* Note: PWM Channel 1 is in single mode as default state and
	* can not be changed to double edge mode */
	PWM_ChannelConfig(LPC_PWM1, 2, PWM_CHANNEL_SINGLE_EDGE);
	PWM_ChannelConfig(LPC_PWM1, 3, PWM_CHANNEL_SINGLE_EDGE);
	PWM_ChannelConfig(LPC_PWM1, 4, PWM_CHANNEL_SINGLE_EDGE);
//	PWM_ChannelConfig(LPC_PWM1, 1, PWM_CHANNEL_SINGLE_EDGE); //Just for channel 2 to 6
	/*if match for channel 1 is 12500000 and PR = 6 then Th = 3 s */
	PWM_MatchUpdate(LPC_PWM1, 2, MR0>>1, PWM_MATCH_UPDATE_NOW);
	PWM_MatchUpdate(LPC_PWM1, 3, MR0>>1, PWM_MATCH_UPDATE_NOW);
	PWM_MatchUpdate(LPC_PWM1, 4, MR0>>1, PWM_MATCH_UPDATE_NOW);
	
	
	PWM_ChannelCmd(LPC_PWM1, 2, ENABLE);
	PWM_ChannelCmd(LPC_PWM1, 3, ENABLE);
	PWM_ChannelCmd(LPC_PWM1, 4, ENABLE);

	PWM_ResetCounter(LPC_PWM1);
	PWM_CounterCmd(LPC_PWM1, ENABLE);
	PWM_Cmd(LPC_PWM1, ENABLE);
	
	
	NVIC_InitStructure.IRQHandler = PWM1_IRQn;  
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.Status = DISABLE;
	NVIC_InitStructure.SubPriority = 4;
	NVIC_Init(&NVIC_InitStructure);
}

void SPI_Config(void)
{

}

/*no usar el uart q coincide con el button key*/
void UART_Config(void)
{
	UART_InitTypeDef UARTConfigStruct;
	UART_FIFO_InitTypeDef UARTFIFOConfigStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART0,ENABLE);
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_UART0,CLKPWR_PCLKSEL_CCLK_DIV_4);
	
	UARTConfigStruct.Baud_rate = 9600;
	UARTConfigStruct.Databits = UART_DATABIT_8;
	UARTConfigStruct.Parity = UART_PARITY_NONE;
	UARTConfigStruct.Stopbits = UART_STOPBIT_1; 

	UART_Init(LPC_UART0, &UARTConfigStruct);

	UARTFIFOConfigStruct.FIFO_DMAMode = DISABLE;
	UARTFIFOConfigStruct.FIFO_Level = UART_FIFO_TRGLEV0;
	UARTFIFOConfigStruct.FIFO_ResetRxBuf = ENABLE;
	UARTFIFOConfigStruct.FIFO_ResetTxBuf = ENABLE;

  UART_FIFOConfig(LPC_UART0, &UARTFIFOConfigStruct);

	UART_TxCmd(LPC_UART0, ENABLE);	
	
// wait for current transmission complete - THR must be empty
//while (!(LPC_UART0->LSR & UART_LSR_TEMT));
	
	NVIC_InitStructure.IRQHandler = UART0_IRQn;
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.SubPriority = 3;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//	
//	/* Enable UART Rx interrupt */
//	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_RBR, ENABLE);
//	/* Enable UART line status interrupt */
//	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_RLS, ENABLE);		
}

void NVIC_Config(void)
{
	NVIC_SetPriorityGrouping(0x07);/*3 bits: 2(5p 0s) 3(4p 1s) 4(3p 2s) 5(2p 3s)(default) 6(1p 4s) 7(0p 5s)  */
}

void Systick_Config(void)
{
	if(SysTick_Config(100000)) while(1);
}

void USB_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	USB_Disconnect();
	
	delay(100);
	
	NVIC_InitStructure.IRQHandler = USB_IRQn;
	NVIC_InitStructure.SubPriority = 7;
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USB_Init();
	USB_Connect(TRUE);
	while (!USB_Configuration) ;
}

void Peripheral(void)
{
	NVIC_Config();
	
  Systick_Config();

	IO_Config();
	
//	ADC_Config();
	
//	TIM_Config();
	
	PWM_Config();
	
	USB_Config();
	
//	UART_Config();
	
//	IWDG_Config();
	
}

/******************************************* SOME PERIPHERAL FUNCTIONS ************************************************/

void USB_Disconnect(void)
{
	Output(0,1<<29|1<<30);	
	
	GPIO_Init(0,29,GPIO_DEFAULT,GPIO_MODE_PU,GPIO_NO_OD);
	GPIO_Init(0,30,GPIO_DEFAULT,GPIO_MODE_PU,GPIO_NO_OD);
	
	Low(0,1<<29|1<<30);
}

/*if uart1, cast (LPC_UART1_TypeDef *) */
void Uart_Tx(LPC_UART_TypeDef* uart ,char * format,...)
{
	va_list arglist;
	va_start(arglist,format);
	vsnprintf(UART_Buffer,sizeof(UART_Buffer),format,arglist);
	UART_Send(uart,(uint8_t*)&UART_Buffer[0],strlen(UART_Buffer),BLOCKING);
	va_end(arglist);
}


static void USB_Transmit(uint8_t* buf,unsigned int len )
{
	uint32_t k;
	uint32_t total = len;
	uint32_t packets = total / 63;
	uint32_t rest_byte = total;
	
	for(k=0; k<=packets ; k++)
	{
		if(rest_byte > 63)
		{
			All_Packets_Sent = 1;
			USB_WriteEP (CDC_DEP_IN, &buf[total - rest_byte], 63);
			rest_byte -=63;
			while(All_Packets_Sent);
		}
		else
		{
			All_Packets_Sent = 1;
			USB_WriteEP (CDC_DEP_IN, &buf[total - rest_byte], rest_byte);
			while(All_Packets_Sent);
			break;
		}
	}
}
void USB_Send_Bytes(uint8_t * stream, unsigned int len)
{
	USB_Transmit((uint8_t*)stream,len);
}

void USB_Trace(char* string, ...)
{
	va_list arglist;
	va_start(arglist,string);
	vsnprintf(USB_Buffer,sizeof(USB_Buffer),string, arglist);
	strcat(USB_Buffer,"\r\n");
	USB_Transmit((uint8_t*)USB_Buffer,strlen(USB_Buffer));
	memset(USB_Buffer,0x00,sizeof(USB_Buffer));
	va_end(arglist);
}


__ASM void delay(unsigned int d)  /*This delay must be tested*/
{
	ldr r1,=16666		// Load immediate and ...
	mul r0,r1				// ... multiply it with tick argument
loop
	subs r0, #1			// In each loop decrement
	bne loop				// until r0 == 0
	bx lr
}

void Delay(uint32_t delay)
{
	DelayC = delay;
	while(DelayC);
}



