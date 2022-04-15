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


void UART_Config(void)
{
	UART_InitTypeDef UARTConfigStruct;
	UART_FIFO_InitTypeDef UARTFIFOConfigStruct;
	Output(0,TXD2|RXD2);
	GPIO_Init(0,10,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD);	
	GPIO_Init(0,11,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD);
	
	UARTConfigStruct.Baud_rate = 38400;
	UARTConfigStruct.Databits = UART_DATABIT_8;
	UARTConfigStruct.Parity = UART_PARITY_NONE;
	UARTConfigStruct.Stopbits = UART_STOPBIT_1;
	
	UART_Init((LPC_UART_TypeDef *)LPC_UART2, &UARTConfigStruct);

	UARTFIFOConfigStruct.FIFO_DMAMode = DISABLE;
	UARTFIFOConfigStruct.FIFO_Level = UART_FIFO_TRGLEV0;
	UARTFIFOConfigStruct.FIFO_ResetRxBuf = ENABLE;
	UARTFIFOConfigStruct.FIFO_ResetTxBuf = ENABLE;
	
	UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART2, &UARTFIFOConfigStruct);

	/* Enable UART Transmit*/
	UART_TxCmd((LPC_UART_TypeDef *)LPC_UART2, ENABLE);
  /* Enable UART Rx interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART2, UART_INTCFG_RBR, DISABLE);
	/* Enable UART line status interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART2, UART_INTCFG_RLS, DISABLE);
	
}

void IO_Config(void)
{	
	/******* Outputs ********/
	Output(2,LED);
	Output(0,INA|INB);
	Output(1,GND_1|GND_2);
	High(0,INA|INB);
	/****** Input IRQ **********/
	Input(2,INT);
	Input(1,FC_1|FC_2);
}

void PWM_Config(void)
{
	PWM_TIMERCFG_Type PWMCfgDat;
	PWM_MATCHCFG_Type PWMMatchCfgDat;

	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCPWM1,ENABLE);
	CLKPWR_SetPCLKDiv (CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_2);
	
	/*Pin Config*/
	Output(2,1<<1);
	GPIO_Init(2,1,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD); //channel 2 pin 74
	
	PWMCfgDat.PrescaleValue = 1;
	PWM_Init(LPC_PWM1, PWM_MODE_TIMER, (void *) &PWMCfgDat); //PCLK = 50 MHz
	
	PWM_MatchUpdate(LPC_PWM1, 0, 50000, PWM_MATCH_UPDATE_NOW);

	PWMMatchCfgDat.IntOnMatch = ENABLE;
	PWMMatchCfgDat.MatchChannel = 0;
	PWMMatchCfgDat.ResetOnMatch = ENABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);
		 
	PWM_ChannelConfig(LPC_PWM1, 2, PWM_CHANNEL_SINGLE_EDGE);
	PWM_MatchUpdate(LPC_PWM1, 2, 0, PWM_MATCH_UPDATE_NOW);
	PWM_ChannelCmd(LPC_PWM1, 2, ENABLE);

	PWM_ResetCounter(LPC_PWM1);
	PWM_CounterCmd(LPC_PWM1, ENABLE);
	PWM_Cmd(LPC_PWM1, ENABLE);
	
	LPC_PWM1->MCR &= ~0x01;
}

void QEI_Config(void)
{
	QEI_CFG_Type QEIConfig;
	QEI_RELOADCFG_Type ReloadConfig;
	NVIC_InitTypeDef NVIC_InitStructure;

	CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCQEI, ENABLE);
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_QEI, CLKPWR_PCLKSEL_CCLK_DIV_1);
	
	QEIConfig.CaptureMode = QEI_CAPMODE_4X;
	QEIConfig.DirectionInvert = QEI_DIRINV_NONE;
	QEIConfig.InvertIndex = QEI_INVINX_NONE;
	QEIConfig.SignalMode = QEI_SIGNALMODE_QUAD;
	
	QEI_Init(LPC_QEI, &QEIConfig);
	
	Input(1,PHB|PHA);
	GPIO_Init(1,20,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD);   // MCI0  P1.20  PhA   Pin 34		LV3
	GPIO_Init(1,23,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD);   // MCI1  P1.23  PhB	 Pin 37		LV4
	
	// Set timer reload value for  QEI that used to set velocity capture period
	ReloadConfig.ReloadOption = QEI_TIMERRELOAD_USVAL;
	ReloadConfig.ReloadValue = CAP_PERIOD;
	QEI_SetTimerReload(LPC_QEI, &ReloadConfig);

	/*All related to position according datasheet*/
	QEI_SetMaxPosition(LPC_QEI, 1200000);
	QEI_SetPositionComp(LPC_QEI,QEI_COMPPOS_CH_0,1500);
	/***********************************************/

	NVIC_InitStructure.IRQHandler = QEI_IRQn;      
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_InitStructure.SubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	// Enable interrupt for velocity Timer overflow for capture velocity into Acc */
	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_TIM_Int, DISABLE);
	// Enable interrupt for direction change */
	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_DIR_Int, ENABLE);	
	// Enable interrupt for position */
	// QEI_IntCmd(LPC_QEI, QEI_INTFLAG_POS0_Int, ENABLE);	
}


void TIM_Config(void)
{
	TIM_MATCH_InitTypeDef MatchStructure;
	TIM_TIMER_InitTypeDef TimerStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM0,ENABLE);
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0,CLKPWR_PCLKSEL_CCLK_DIV_4);
	
	TimerStructure.PrescaleValue = 1;	
	TimerStructure.CounterOption = TIM_MODE_TIMER;
	TimerStructure.CountInputSelect = TIM_COUNTER_INCAP0;
	
	MatchStructure.IntOnMatch = ENABLE;
	MatchStructure.StopOnMatch = DISABLE;
	MatchStructure.ResetOnMatch = ENABLE;
	MatchStructure.MatchChannel = 0;  
	MatchStructure.MatchValue = 2500000; 
	MatchStructure.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	
	TIM_Init(LPC_TIM0,&TimerStructure);
	TIM_ConfigMatch(LPC_TIM0,&MatchStructure);
	
	TIM_Cmd(LPC_TIM0,ENABLE);
	
	NVIC_InitStructure.IRQHandler = TIMER0_IRQn;
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.SubPriority = 4;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void Systick_Config(void)
{
	if(SysTick_Config(100)) while(1); //us
}

void USB_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	USB_Disconnect();
	
	delay(100);
	
	NVIC_InitStructure.IRQHandler = USB_IRQn;
	NVIC_InitStructure.SubPriority = 3;
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USB_Init();
	USB_Connect(TRUE);
	while (!USB_Configuration) ;
}




/**********************************************************************************************************************/
/******************************************* SOME PERIPHERAL FUNCTIONS ************************************************/
/**********************************************************************************************************************/
void UART_Trace(char* format,...)
{
	uint32_t len,i;
	char* p;
	va_list arglist;
	va_start(arglist,format);
	vsnprintf(UART_Buffer,sizeof(UART_Buffer),format, arglist);
	len = strlen(UART_Buffer);
	p = UART_Buffer;
	for(i=0;i<len;i++)
	{
		UART_SendByte((LPC_UART_TypeDef *)LPC_UART2, *p++);
		while (UART_CheckBusy((LPC_UART_TypeDef *)LPC_UART2));
	}
	memset(UART_Buffer,0x00,sizeof(UART_Buffer));
	va_end(arglist);	
}

void USB_Disconnect(void)
{
	Output(0,1<<29|1<<30);	
	
	GPIO_Init(0,29,GPIO_DEFAULT,GPIO_MODE_PU,GPIO_NO_OD);
	GPIO_Init(0,30,GPIO_DEFAULT,GPIO_MODE_PU,GPIO_NO_OD);
	
	Low(0,1<<29|1<<30);
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
	USB_Transmit(stream,len);
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


__ASM void delay(unsigned int d)  
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



