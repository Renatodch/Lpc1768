#include "main.h"


extern __IO uint32_t DelayC;

uint32_t c[10] = {20000000,30000000,40000000,50000000,60000000,70000000,80000000,90000000,100000000, 0};

uint8_t cx[10] = {1,2,1,1,1,1,1,1,0,2};
uint8_t cy[10] = {2,2,1,1,1,1,1,1,2,1};
uint8_t cz[10] = {1,2,1,1,1,1,1,1,1,1};

uint32_t MR0;

int i = 0;

void PWM1_IRQHandler(void)
{
	if (PWM_GetIntStatus(LPC_PWM1, PWM_INTSTAT_MR0))
	{
			
			if(MR0 <= 10000) MR0 = 10000;
			else MR0 -= 100 ;
		
			LPC_PWM1->MR0	= MR0;
		
			LPC_PWM1->MR1 = MR0>>1;
			
			LPC_PWM1->MR2 = MR0>>1;
			
			LPC_PWM1->MR3 = MR0>>1;
			
			

			LPC_PWM1->LER |= (PWM_LER_EN_MATCHn_LATCH(0)|PWM_LER_EN_MATCHn_LATCH(1)|PWM_LER_EN_MATCHn_LATCH(2)|PWM_LER_EN_MATCHn_LATCH(3));

			LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
			LPC_PWM1->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;
		
		/* Clear the interrupt flag */
		PWM_ClearIntPending(LPC_PWM1, PWM_INTSTAT_MR0);
	}
}


void EINT3_IRQHandler(void)
{
	if(Read_int(2,1<<10,F))
	{
    EXTI_ClearEXTIFlag(EXTI_EINT2);
		Clr_int(2,1<<10);
//		Toggle(2,1<<0);
	}
}

void SysTick_Handler(void)
{
//	SysTick->CTRL &= ~((uint32_t)(1<<16)); /*When polling underflow(?)*/
	SystemTime_Timer();
	if(DelayC != 0) DelayC --;	
}

void TIMER0_IRQHandler(void)
{
	if(TIM_GetIntStatus(LPC_TIM0, TIM_INT_MR0))
	{
//		Toggle(2,1<<0);
		TIM_ClearIntPending(LPC_TIM0, TIM_INT_MR0);
	}
}



void UART0_IRQHandler(void) /*ver uart.c*/
{
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId(LPC_UART0);
	tmp = intsrc & UART_IIR_INTID_MASK;

	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = UART_GetLineStatus(LPC_UART0);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);
		if (tmp1)	while(1); //ERROR EXIST		
	}
	// Receive Data Available or Character time-out
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
			//UART_IntReceive();
	}
	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE){
			//UART_IntTransmit();
	}
}















