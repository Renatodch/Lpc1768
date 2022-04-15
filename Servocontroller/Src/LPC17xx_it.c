#include "main.h"

/** Velocity Accumulator */
extern __IO uint64_t VeloAcc;
extern __IO uint32_t VeloCapCnt;
extern __IO FlagStatus VeloAccFlag;
extern uint32_t averageVelo;

extern __IO uint32_t DelayC;
extern Trayectoria Eje;

void TIMER0_IRQHandler(void)
{
	if(TIM_GetIntStatus(LPC_TIM0,TIM_INT_MR0))
	{
		if(Eje.Ejecutando)
		{
			UART_Trace("%c%07d%c%07d",23,0,24,LPC_QEI->QEIPOS);	
		}
		TIM_ClearIntPending(LPC_TIM0,TIM_INT_MR0);
	}
}

void QEI_IRQHandler(void)
{
	// Check whether if velocity timer overflow
	if (QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_TIM_Int) == SET) {
		
		if (VeloAccFlag == RESET) {

			// Get current velocity captured and update to accumulate
			VeloAcc += QEI_GetVelocityCap(LPC_QEI);

			// Update Velocity capture times
			VeloAccFlag = ((VeloCapCnt++) >= MAX_CAP_TIMES) ? SET : RESET;
		}
		
		if(VeloAccFlag == SET)
		{
			averageVelo = (uint32_t)(VeloAcc / VeloCapCnt);
				
			Eje.Prismatica.RPM = QEI_CalculateRPM(LPC_QEI, averageVelo, ENC_RES);

			UART_Trace("%c%07d%c%07d",23,Eje.Prismatica.DC,24,Eje.Prismatica.RPM);
			
			VeloAccFlag = RESET;
			VeloAcc = 0;
			VeloCapCnt = 0;
		}
		QEI_IntClear(LPC_QEI, QEI_INTFLAG_TIM_Int);
	}

	// Check whether if direction change occurred
	else if (QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_DIR_Int) == SET) {
	
		QEI_IntClear(LPC_QEI, QEI_INTFLAG_DIR_Int);
	}
	
	else if (QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_POS0_Int) == SET)
	{
		QEI_IntClear(LPC_QEI, QEI_INTFLAG_POS0_Int);	
	}
}

void EINT3_IRQHandler(void)
{
	if(Read_int(2,1<<10,F))
	{
    EXTI_ClearEXTIFlag(EXTI_EINT2);
		Clr_int(2,1<<10);
		Toggle(2,1<<0);
	}
}

void SysTick_Handler(void)
{
	SystemTime_Timer();
	if(DelayC != 0) DelayC --;	
}












