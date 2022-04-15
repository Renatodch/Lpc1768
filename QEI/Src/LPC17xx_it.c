#include "main.h"

/** Velocity Accumulator */
extern __IO uint64_t VeloAcc;
/** Times of Velocity capture */
extern __IO uint32_t VeloCapCnt;
/** Flag indicates Times of Velocity capture is enough to read out */
extern __IO FlagStatus VeloAccFlag;

extern __IO uint32_t DelayC;


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
		// Reset Interrupt flag pending
		QEI_IntClear(LPC_QEI, QEI_INTFLAG_TIM_Int);
	}

	// Check whether if direction change occurred
	else if (QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_DIR_Int) == SET) {

		// Reset Interrupt flag pending
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
//	SysTick->CTRL &= ~((uint32_t)(1<<16)); /*When polling underflow(?)*/
	SystemTime_Timer();
	if(DelayC != 0) DelayC --;	
}

uint32_t c[10] = {100000000,50000000,25000000,50000000,60000000,70000000,80000000,90000000,100000000, 0};

uint8_t cx[10] = {2,2,1,1,1,1,1,1,0,2};
uint8_t cy[10] = {2,0,2,1,1,1,1,1,2,1};
uint8_t cz[10] = {1,0,1,1,1,1,1,1,1,1};
uint8_t px = 0;
uint8_t py = 0;
uint8_t pz = 0;
uint32_t ind = 0;

void PWM1_IRQHandler(void)
{
	if (PWM_GetIntStatus(LPC_PWM1, PWM_INTSTAT_MR0))
	{
		if( px || py || pz ) 
		{
			LPC_PWM1->MR0 = c[ind];			
			
			if(px)
			{
				LPC_PWM1->MR2 = c[ind] >> 1;
				px--;
			}
			else LPC_PWM1->MR2 =  0;
			if(py)
			{
				LPC_PWM1->MR3 = c[ind] >> 1;
				py--;
			}
			else LPC_PWM1->MR3 =  0;		
			if(pz)
			{
				LPC_PWM1->MR4 = c[ind] >> 1;
				pz--;
			}
			else LPC_PWM1->MR4 =  0;
			
			LPC_PWM1->LER |= (1<<0 | 1<<2 | 1<<3 | 1<<4);
			LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
			LPC_PWM1->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;			

		}
		else
		{
			ind++;
			
			if(ind == 3)
			{
				ind = px = py = pz= 0;
				
				LPC_PWM1->MR2 = LPC_PWM1->MR3 = LPC_PWM1->MR4 =  0;	
				LPC_PWM1->LER |= (1<<0 | 1<<2 | 1<<3 | 1<<4);
				LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
				LPC_PWM1->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;
				
				LPC_PWM1->MCR &= ~0x01;	
			}
			else
			{	
				px = cx[ind];
				py = cy[ind];
				pz = cz[ind];					
				
				while(!px && !py && !pz)
				{
					ind ++;
					px = cx[ind];
					py = cy[ind];
					pz = cz[ind];	
				}
				
				LPC_PWM1->MR0 = c[ind];
				
				if(px)
				{
					LPC_PWM1->MR2 = c[ind] >> 1;
					px--;
				}
				else LPC_PWM1->MR2 =  0;
				if(py)
				{
					LPC_PWM1->MR3 = c[ind] >> 1;
					py--;
				}
				else LPC_PWM1->MR3 =  0;		
				if(pz)
				{
					LPC_PWM1->MR4 = c[ind] >> 1;
					pz--;
				}
				else LPC_PWM1->MR4 =  0;
				
				LPC_PWM1->LER |= (1<<0 | 1<<2 | 1<<3 | 1<<4);
				LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
				LPC_PWM1->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;	
			}
		}
		PWM_ClearIntPending(LPC_PWM1,PWM_INTSTAT_MR0);
	}
}

















