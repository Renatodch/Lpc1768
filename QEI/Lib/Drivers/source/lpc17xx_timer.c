/**********************************************************************
* $Id$		lpc17xx_timer.c				2018-04-2
*//**
* @file		lpc17xx_timer.c
* @brief	Contains all functions support for Timer firmware library
* 			on LPC17xx
* @version	3.1
* @date		2. April. 2018
* @author	RenatoDc
*
* Copyright(C) 2011, NXP Semiconductor
* All rights reserved.
***********************************************************************
**********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @addtogroup TIM
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_timer.h"
#include "lpc17xx_clkpwr.h"



/* Private Functions ---------------------------------------------------------- */


/* End of Private Functions ---------------------------------------------------- */


/*********************************************************************//**
 * @brief 		Get Interrupt Status
 * @param[in]	TIMx Timer selection, should be:
 *   			- LPC_TIM0: TIMER0 peripheral
 * 				- LPC_TIM1: TIMER1 peripheral
 * 				- LPC_TIM2: TIMER2 peripheral
 * 				- LPC_TIM3: TIMER3 peripheral
 * @param[in]	IntFlag: interrupt type, should be:
 * 				- TIM_MR0_INT: Interrupt for Match channel 0
 * 				- TIM_MR1_INT: Interrupt for Match channel 1
 * 				- TIM_MR2_INT: Interrupt for Match channel 2
 * 				- TIM_MR3_INT: Interrupt for Match channel 3
 * 				- TIM_CR0_INT: Interrupt for Capture channel 0
 * 				- TIM_CR1_INT: Interrupt for Capture channel 1
 * @return 		FlagStatus
 * 				- SET : interrupt
 * 				- RESET : no interrupt
 **********************************************************************/
FlagStatus TIM_GetIntStatus(LPC_TIM_TypeDef *TIMx, TIM_INT_E IntFlag)
{
	if (TIMx->IR & (uint32_t)IntFlag)	return SET;
	return RESET;
}
/*********************************************************************//**
 * @brief 		Get Capture Interrupt Status
 * @param[in]	TIMx Timer selection, should be:
 *  	   		- LPC_TIM0: TIMER0 peripheral
 * 				- LPC_TIM1: TIMER1 peripheral
 * 				- LPC_TIM2: TIMER2 peripheral
 * 				- LPC_TIM3: TIMER3 peripheral
 * @param[in]	IntFlag: interrupt type, should be:
 * 				- TIM_MR0_INT: Interrupt for Match channel 1<<0
 * 				- TIM_MR1_INT: Interrupt for Match channel 1<<1
 * 				- TIM_MR2_INT: Interrupt for Match channel 1<<2
 * 				- TIM_MR3_INT: Interrupt for Match channel 1<<3
 * 				- TIM_CR0_INT: Interrupt for Capture channel 1<<4
 * 				- TIM_CR1_INT: Interrupt for Capture channel 1<<5
 * @return 		FlagStatus
 * 				- SET : interrupt
 * 				- RESET : no interrupt
 **********************************************************************/
FlagStatus TIM_GetIntCaptureStatus(LPC_TIM_TypeDef *TIMx, TIM_INT_E IntFlag)
{
	if((TIMx->IR) & (uint32_t)IntFlag) return SET;
	return RESET;
}
/*********************************************************************//**
 * @brief 		Clear Interrupt pending
 * @param[in]	TIMx Timer selection, should be:
 *    			- LPC_TIM0: TIMER0 peripheral
 * 				- LPC_TIM1: TIMER1 peripheral
 * 				- LPC_TIM2: TIMER2 peripheral
 * 				- LPC_TIM3: TIMER3 peripheral
 * @param[in]	IntFlag: interrupt type, should be:
 * 				- TIM_MR0_INT: Interrupt for Match channel 0
 * 				- TIM_MR1_INT: Interrupt for Match channel 1
 * 				- TIM_MR2_INT: Interrupt for Match channel 2
 * 				- TIM_MR3_INT: Interrupt for Match channel 3
 * 				- TIM_CR0_INT: Interrupt for Capture channel 0
 * 				- TIM_CR1_INT: Interrupt for Capture channel 1
 * @return 		None
 **********************************************************************/
void TIM_ClearIntPending(LPC_TIM_TypeDef *TIMx, TIM_INT_E IntFlag)
{
	TIMx->IR |= (uint32_t)IntFlag;
}

/*********************************************************************//**
 * @brief 		Clear Capture Interrupt pending
 * @param[in]	TIMx Timer selection, should be
 *    			- LPC_TIM0: TIMER0 peripheral
 * 				- LPC_TIM1: TIMER1 peripheral
 * 				- LPC_TIM2: TIMER2 peripheral
 * 				- LPC_TIM3: TIMER3 peripheral
 * @param[in]	IntFlag interrupt type, should be:
 *				- TIM_MR0_INT: Interrupt for Match channel 0
 * 				- TIM_MR1_INT: Interrupt for Match channel 1
 * 				- TIM_MR2_INT: Interrupt for Match channel 2
 * 				- TIM_MR3_INT: Interrupt for Match channel 3
 * 				- TIM_CR0_INT: Interrupt for Capture channel 0
 * 				- TIM_CR1_INT: Interrupt for Capture channel 1
 * @return 		None
 **********************************************************************/
void TIM_ClearIntCapturePending(LPC_TIM_TypeDef *TIMx, TIM_INT_E IntFlag)
{
	TIMx->IR |= (uint32_t)IntFlag;
}


/*********************************************************************//**
 * @brief 		Initial Timer/Counter device
 * 				 	Set Clock frequency for Timer
 * 					Set initial configuration for Timer
 * @param[in]	TIMx  Timer selection, should be:
 * 				- LPC_TIM0: TIMER0 peripheral
 * 				- LPC_TIM1: TIMER1 peripheral
 * 				- LPC_TIM2: TIMER2 peripheral
 * 				- LPC_TIM3: TIMER3 peripheral
 * @param[in]	TimerCounterMode Timer counter mode, should be:
 * 				- TIM_TIMER_MODE: Timer mode
 * 				- TIM_COUNTER_RISING_MODE: Counter rising mode
 * 				- TIM_COUNTER_FALLING_MODE: Counter falling mode
 * 				- TIM_COUNTER_ANY_MODE:Counter on both edges
 * @param[in]	TIM_ConfigStruct pointer to TIM_TIMERCFG_Type
 * 				that contains the configuration information for the
 *                    specified Timer peripheral.
 * @return 		None
 **********************************************************************/
void TIM_Init(LPC_TIM_TypeDef *TIMx, TIM_TIMER_InitTypeDef *TIM_ConfigStruct)
{
	/*Timer as Default*/
	TIMx->CTCR &= ~(uint32_t)(0x0F);
	
	TIMx->TC =0;
	TIMx->PC =0;
	TIMx->PR =0;
	TIMx->TCR |= TIM_RESET; //Reset Counter
	TIMx->TCR &= ~TIM_RESET; //release reset
	
	if (TIM_ConfigStruct->CounterOption == TIM_MODE_TIMER)
	{
	  TIMx->PR   = TIM_ConfigStruct->PrescaleValue - 1;      // 0 === 1 
	}
	else /*Counter Mode Selected: rising falling or both edge*/
	{
		TIMx->CCR &= ~TIM_CCR_CHANNEL_MASKBIT(TIM_ConfigStruct->CountInputSelect); /**/
		TIMx->CTCR |= (uint32_t)(TIM_ConfigStruct->CounterOption);
		TIMx->CTCR |= (uint32_t)(TIM_ConfigStruct->CountInputSelect << 2);
	}

	// Clear interrupt pending
	TIMx->IR = 0xFFFFFFFF;
}

/*********************************************************************//**
 * @brief 		Reset Timer/Counter device,
 * 					Make TC and PC are synchronously reset on the next
 * 					positive edge of PCLK
 * @param[in]	TIMx Pointer to timer device, should be:
 *   			- LPC_TIM0: TIMER0 peripheral
 * 				- LPC_TIM1: TIMER1 peripheral
 * 				- LPC_TIM2: TIMER2 peripheral
 * 				- LPC_TIM3: TIMER3 peripheral
 * @return 		None
 **********************************************************************/
void TIM_ResetCounter(LPC_TIM_TypeDef *TIMx)
{
	TIMx->TCR |= TIM_RESET;
	TIMx->TCR &= ~TIM_RESET;
}

/*********************************************************************//**
 * @brief 		Configuration for Match register
 * @param[in]	TIMx Pointer to timer device, should be:
 *   			- LPC_TIM0: TIMER0 peripheral
 * 				- LPC_TIM1: TIMER1 peripheral
 * 				- LPC_TIM2: TIMER2 peripheral
 * 				- LPC_TIM3: TIMER3 peripheral
 * @param[in]   TIM_MatchConfigStruct Pointer to TIM_MATCHCFG_Type
 * 					- MatchChannel : choose channel 0 or 1
 * 					- IntOnMatch	 : if SET, interrupt will be generated when MRxx match
 * 									the value in TC
 * 					- StopOnMatch	 : if SET, TC and PC will be stopped whenM Rxx match
 * 									the value in TC
 * 					- ResetOnMatch : if SET, Reset on MR0 when MRxx match
 * 									the value in TC
 * 					-ExtMatchOutputType: Select output for external match
 * 						 +	 0:	Do nothing for external output pin if match
 *						 +   1:	Force external output pin to low if match
 *						 + 	 2: Force external output pin to high if match
 *						 + 	 3: Toggle external output pin if match
 *					MatchValue: Set the value to be compared with TC value
 * @return 		None
 **********************************************************************/
void TIM_ConfigMatch(LPC_TIM_TypeDef *TIMx, TIM_MATCH_InitTypeDef *TIM_MatchCfgStruct)
{
	switch(TIM_MatchCfgStruct->MatchChannel)
	{
	case 0:	TIMx->MR0 = TIM_MatchCfgStruct->MatchValue; break;
	case 1: TIMx->MR1 = TIM_MatchCfgStruct->MatchValue; break;
	case 2: TIMx->MR2 = TIM_MatchCfgStruct->MatchValue; break;
	case 3: TIMx->MR3 = TIM_MatchCfgStruct->MatchValue; break;
	default: while(1);
	}
	TIMx->MCR &=~TIM_MCR_CHANNEL_MASKBIT(TIM_MatchCfgStruct->MatchChannel);
	
	//interrupt on MRn
	if (TIM_MatchCfgStruct->IntOnMatch)
		TIMx->MCR |= TIM_INT_ON_MATCH(TIM_MatchCfgStruct->MatchChannel);

	//reset on MRn
	if (TIM_MatchCfgStruct->ResetOnMatch)
		TIMx->MCR |= TIM_RESET_ON_MATCH(TIM_MatchCfgStruct->MatchChannel);

	//stop on MRn
	if (TIM_MatchCfgStruct->StopOnMatch)
		TIMx->MCR |= TIM_STOP_ON_MATCH(TIM_MatchCfgStruct->MatchChannel);

	// match output type
	TIMx->EMR 	&= ~TIM_EM_MASK(TIM_MatchCfgStruct->MatchChannel);  /*Reset TIMx->EMR.EMCx bit*/
	TIMx->EMR   |= TIM_EM_SET(TIM_MatchCfgStruct->MatchChannel,TIM_MatchCfgStruct->ExtMatchOutputType);
	/*TIMx->EMR.EMx bit is Set, Clear, Toggle or nothing depending of EMCx[x:x]*/
}

/*********************************************************************//**
 * @brief 		Configuration for Capture register
 * @param[in]	TIMx Pointer to timer device, should be:
 *   			- LPC_TIM0: TIMER0 peripheral
 * 				- LPC_TIM1: TIMER1 peripheral
 * 				- LPC_TIM2: TIMER2 peripheral
 * 				- LPC_TIM3: TIMER3 peripheral
 * 					- CaptureChannel: set the channel to capture data
 * 					- RisingEdge    : if SET, Capture at rising edge
 * 					- FallingEdge	: if SET, Capture at falling edge
 * 					- IntOnCaption  : if SET, Capture generate interrupt
 * @param[in]   TIM_CaptureConfigStruct	Pointer to TIM_CAPTURECFG_Type
 * @return 		None
 **********************************************************************/
void TIM_ConfigCapture(LPC_TIM_TypeDef *TIMx, TIM_CAPTURE_InitTypeDef *TIM_CaptureConfigStruct)
{

	TIMx->CCR &= ~TIM_CCR_CHANNEL_MASKBIT(TIM_CaptureConfigStruct->CaptureChannel);

	if (TIM_CaptureConfigStruct->RisingEdge == ENABLE)
		TIMx->CCR |= TIM_CAP_RISING(TIM_CaptureConfigStruct->CaptureChannel);

	if (TIM_CaptureConfigStruct->FallingEdge == ENABLE)
		TIMx->CCR |= TIM_CAP_FALLING(TIM_CaptureConfigStruct->CaptureChannel);

	if (TIM_CaptureConfigStruct->IntOnCaption == ENABLE)
		TIMx->CCR |= TIM_INT_ON_CAP(TIM_CaptureConfigStruct->CaptureChannel);
}

/*********************************************************************//**
 * @brief 		Read value of capture register in timer/counter device
 * @param[in]	TIMx Pointer to timer/counter device, should be:
 *  			- LPC_TIM0: TIMER0 peripheral
 * 				- LPC_TIM1: TIMER1 peripheral
 * 				- LPC_TIM2: TIMER2 peripheral
 * 				- LPC_TIM3: TIMER3 peripheral
 * @param[in]	CaptureChannel: capture channel number, should be:
 * 				- TIM_COUNTER_INCAP0: CAPn.0 input pin for TIMERn
 * 				- TIM_COUNTER_INCAP1: CAPn.1 input pin for TIMERn
 * @return 		Value of capture register
 **********************************************************************/
uint32_t TIM_GetCaptureValue(LPC_TIM_TypeDef *TIMx, TIM_COUNTER_INPUT_E CaptureChannel)
{
	if(CaptureChannel==0)	return TIMx->CR0;
	else									return TIMx->CR1;
}
void TIM_Cmd(LPC_TIM_TypeDef *TIMx, FunctionalState NewState)
{
	if (NewState == ENABLE)	TIMx->TCR	|=  TIM_ENABLE;
	else										TIMx->TCR &= ~TIM_ENABLE;
}
void TIM_UpdateMatchValue(LPC_TIM_TypeDef *TIMx, uint8_t channel, uint32_t value)
{
	switch(channel)
	{
	case 0:	TIMx->MR0 = value;	break;
	case 1: TIMx->MR1 = value;	break;
	case 2:	TIMx->MR2 = value;	break;
	case 3:	TIMx->MR3 = value;	break;
	default:/*Error Loop*/ while(1);
	}
}
/**
 * @}
 */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
