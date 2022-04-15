#include "main.h"



void SysTick_Handler(void)
{
	//Clear System Tick counter flag
	SYSTICK_ClearCounterFlag();
	//toggle P0.0
	toggle(0, Pin(0));
		
}

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















