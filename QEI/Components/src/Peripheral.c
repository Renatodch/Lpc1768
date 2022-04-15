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


void IO_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	Turn_On(CLKPWR_PCONP_PCGPIO);
	
	/******* Outputs ********/
	Output(2,LED);
	High(2,LED);
	GPIO_Init(2,0,GPIO_DEFAULT,GPIO_MODE_PU,GPIO_NO_OD);
	
	/****** Input IRQ **********/
	Input(2,INT);
	Set_edge(2,INT,F);
	Clr_int(2,INT);	
	GPIO_Init(2,10,GPIO_DEFAULT,GPIO_MODE_PU,GPIO_NO_OD);
	
	NVIC_InitStructure.IRQHandler = EINT3_IRQn;      /*Port 0-2 As basis, works with EINT3*/
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_InitStructure.SubPriority = 5;
	NVIC_Init(&NVIC_InitStructure);

}

void QEI_Config(void)
{
	QEI_CFG_Type QEIConfig;
	QEI_RELOADCFG_Type ReloadConfig;
	NVIC_InitTypeDef NVIC_InitStructure;

	CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCQEI, ENABLE);
	/* As default, peripheral clock for QEI module
	 * is set to FCCLK / 2 */
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_QEI, CLKPWR_PCLKSEL_CCLK_DIV_1);
	
	QEIConfig.CaptureMode = QEI_CAPMODE_4X;
	QEIConfig.DirectionInvert = QEI_DIRINV_NONE;
	QEIConfig.InvertIndex = QEI_INVINX_NONE;
	QEIConfig.SignalMode = QEI_SIGNALMODE_QUAD;
	
	QEI_Init(LPC_QEI, &QEIConfig);
	
	Input(1,1<<20|1<<23|1<<24);
	GPIO_Init(1,20,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD);   // MCI0  P1.20  PhA   Pin 34
	GPIO_Init(1,23,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD);   // MCI1  P1.23  PhB	 Pin 37
	GPIO_Init(1,24,GPIO_FUNC_1,GPIO_MODE_PU,GPIO_NO_OD);   // MCI2  P1.24  Index Pin 38
	
	// Set timer reload value for  QEI that used to set velocity capture period
	ReloadConfig.ReloadOption = QEI_TIMERRELOAD_USVAL;
	ReloadConfig.ReloadValue = CAP_PERIOD;
	QEI_SetTimerReload(LPC_QEI, &ReloadConfig);

	/*All related to position according to datasheet*/
	QEI_SetMaxPosition(LPC_QEI, 100000);
	QEI_SetPositionComp(LPC_QEI,QEI_COMPPOS_CH_0,1000);
	/***********************************************/

	NVIC_InitStructure.IRQHandler = QEI_IRQn;      
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_InitStructure.SubPriority = 8;
	NVIC_Init(&NVIC_InitStructure);

	// Enable interrupt for velocity Timer overflow for capture velocity into Acc */
	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_TIM_Int, ENABLE);
	// Enable interrupt for direction change */
	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_DIR_Int, ENABLE);	
	// Enable interrupt for position */
//	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_POS0_Int, ENABLE);	
//	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_POS1_Int, ENABLE);	
//	QEI_IntCmd(LPC_QEI, QEI_INTFLAG_POS2_Int, ENABLE);	
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
	PWM_MatchUpdate(LPC_PWM1, 0, 100000000, PWM_MATCH_UPDATE_NOW);
	/* PWM Timer/Counter will be reset when channel 0 matching
	 * Enable interrupt when match
	 * no stop when match */
	PWMMatchCfgDat.IntOnMatch = ENABLE;
	PWMMatchCfgDat.MatchChannel = 0;
	PWMMatchCfgDat.ResetOnMatch = ENABLE;
	PWMMatchCfgDat.StopOnMatch = DISABLE;
	PWM_ConfigMatch(LPC_PWM1, &PWMMatchCfgDat);
		 

	PWM_ChannelConfig(LPC_PWM1, 2, PWM_CHANNEL_SINGLE_EDGE);
	PWM_ChannelConfig(LPC_PWM1, 3, PWM_CHANNEL_SINGLE_EDGE);
	PWM_ChannelConfig(LPC_PWM1, 4, PWM_CHANNEL_SINGLE_EDGE);
	
	PWM_MatchUpdate(LPC_PWM1, 2, 0, PWM_MATCH_UPDATE_NOW);
	PWM_MatchUpdate(LPC_PWM1, 3, 0, PWM_MATCH_UPDATE_NOW);
	PWM_MatchUpdate(LPC_PWM1, 4, 0, PWM_MATCH_UPDATE_NOW);

	PWM_ChannelCmd(LPC_PWM1, 2, ENABLE);
	PWM_ChannelCmd(LPC_PWM1, 3, ENABLE);
	PWM_ChannelCmd(LPC_PWM1, 4, ENABLE);

	PWM_ResetCounter(LPC_PWM1);
	PWM_CounterCmd(LPC_PWM1, ENABLE);
	PWM_Cmd(LPC_PWM1, ENABLE);

	NVIC_InitStructure.IRQHandler = PWM1_IRQn;  
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_InitStructure.SubPriority = 4;
	NVIC_Init(&NVIC_InitStructure);
	
	LPC_PWM1->MCR &= ~0x01;
}


/**********************************************************************************************************************/
/******************************************* SOME PERIPHERAL FUNCTIONS ************************************************/
/**********************************************************************************************************************/

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



