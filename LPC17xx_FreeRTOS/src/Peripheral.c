#include "main.h"

char UART_Buffer[256]={0};


void Config_Peripheral(void)
{
	Clk_Power_Config();
		
	GPIO_Config();

	WDT_Config();
	
	Interrupts_Config();
}







/**** System Control & Clocking and Power Control  *************/

void Clk_Power_Config(void)
{
	/*Turn off peripheral turned on by default*/
	LPC_SC->PCONP &= 0x00000000 & CLKPWR_PCONP_BITMASK; 
	
	/*Turn on these periphs*/
	LPC_SC->PCONP |= (
	CLKPWR_PCONP_PCTIM0|
	CLKPWR_PCONP_PCUART0|
	CLKPWR_PCONP_PCGPIO
	) & CLKPWR_PCONP_BITMASK;
	
	/*All peripheral starts with PCLK = SystemClock / 4. You could change PCLK for your porpuses*/
	/*TIMER 0*/
	//CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0,CLKPWR_PCLKSEL_CCLK_DIV_4);
}

/*********** Interrupts **********/
void Interrupts_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_SetPriorityGrouping(0x07);/*3 bits: 2(5p 0s) 3(4p 1s) 4(3p 2s) 5(2p 3s)(default) 6(1p 4s) 7(0p 5s)  */
	
	NVIC_InitStructure.IRQHandler = UART0_IRQn;
	NVIC_InitStructure.PrePriority = 0;
	NVIC_InitStructure.SubPriority = 32;
	NVIC_InitStructure.Status = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable UART Rx interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_RBR, ENABLE);
	/* Enable UART line status interrupt */
	UART_IntConfig((LPC_UART_TypeDef *)LPC_UART0, UART_INTCFG_RLS, ENABLE);	
}

/******* WDT ********/

void WDT_Config(void)
{
	uint32_t TimeOut = 5000000; /*in uS*/
	WDT_Init(WDT_CLKSRC_PCLK, WDT_MODE_RESET, TimeOut);
}


/************ GPIO & PinConnect Block ***********************/
void GPIO_Config(void)
{
	GPIO_InitTypeDef 		 GpioStructure;
	PINSEL_InitTypeDef   PinStruct;

	/*********Discrete Input Outputs*****************/
	GpioStructure.Mask  	= GPIO_MASK_UNLOCK;
	
	GpioStructure.Dir   	= GPIO_DIR_OUT;
	GpioStructure.Pins  	= _BIT(0)| _BIT(1);
	GpioStructure.Port  	= LPC_GPIO0;
	GpioStructure.State   = GPIO_LVL_HIGH;
	GpioStructure.EdgeInt = GPIO_INT_EDGE_DISABLE;
	GPIO_Init(&GpioStructure);
	
	GpioStructure.Dir   	= GPIO_DIR_IN;
	GpioStructure.EdgeInt = GPIO_INT_EDGE_RISING;
	GpioStructure.Pins  	= _BIT(2)| _BIT(3);
	GPIO_Init(&GpioStructure);
	
	/***************** UART 0 ***********************/
	PinStruct.Funcnum = AF_1;
	PinStruct.OpenDrain = NORMAL;
	PinStruct.Pinmode = PU;
	PinStruct.Portnum = GPIO_0;
	
	PinStruct.Pinnum = 2;
 	Pincon_Init(&PinStruct);
	PinStruct.Pinnum = 3;
 	Pincon_Init(&PinStruct);

  /**************** TIM 0 MATCH **********************/
	PinStruct.Portnum = GPIO_1;
	PinStruct.Pinnum = 28;
 	Pincon_Init(&PinStruct);

}

/************ TIM ******************/

void Timer_Config(void)
{
	//TIM_CAPTURE_InitTypeDef CaptureStructure;
	TIM_MATCH_InitTypeDef MatchStructure;
	TIM_TIMER_InitTypeDef TimerStructure;
	
	TimerStructure.PrescaleValue = 25000;	/*TC increments every 25000 cycles of PCLK = 100MHz/4*/
	TimerStructure.CounterOption = TIM_MODE_TIMER;
	TimerStructure.CountInputSelect = TIM_COUNTER_INCAP0;
	
	MatchStructure.ResetOnMatch = ENABLE;
	MatchStructure.MatchChannel = 0;
	MatchStructure.MatchValue = 500;
	MatchStructure.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	
	TIM_Init(LPC_TIM0,&TimerStructure);
	TIM_ConfigMatch(LPC_TIM0,&MatchStructure);
}

/************** SPI ************************/
void SPI_Config(void)
{

}

/************** UART *********************/
void Uart_Config(void)
{
	UART_InitTypeDef UARTConfigStruct;
	UART_FIFO_InitTypeDef UARTFIFOConfigStruct;

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
}

/*if uart1, cast (LPC_UART1_TypeDef *) */
void Send_Uart_Format(LPC_UART_TypeDef* uart ,char * format,...)
{
	va_list arglist;
	va_start(arglist,format);
	vsnprintf(UART_Buffer,sizeof(UART_Buffer),format,arglist);
	UART_Send(uart,(uint8_t*)&UART_Buffer[0],strlen(UART_Buffer),BLOCKING);
	va_end(arglist);
}









/*###########################################################################################################################*/
/*######################################### IRQ_Handler #####################################################################*/
/*###########################################################################################################################*/


//void UART0_IRQHandler(void)
//{
//	uint32_t intsrc, tmp, tmp1;

//	/* Determine the interrupt source */
//	intsrc = UART_GetIntId(LPC_UART0);
//	tmp = intsrc & UART_IIR_INTID_MASK;

//	// Receive Line Status
//	if (tmp == UART_IIR_INTID_RLS){
//		// Check line status
//		tmp1 = UART_GetLineStatus(LPC_UART0);
//		// Mask out the Receive Ready and Transmit Holding empty status
//		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);
//		if (tmp1)	while(1); //ERROR EXIST		
//	}
//	// Receive Data Available or Character time-out
//	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
//			//UART_IntReceive();
//	}
//	// Transmit Holding Empty
//	if (tmp == UART_IIR_INTID_THRE){
//			//UART_IntTransmit();
//	}
//}



