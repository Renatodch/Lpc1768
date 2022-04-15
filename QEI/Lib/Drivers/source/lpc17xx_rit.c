/**********************************************************************
* $Id$		lpc17xx_rit.c				2018-04-03
*//**
* @file		lpc17xx_rit.c
* @brief	Contains all functions support for RIT firmware library on LPC17xx
* @version	2.0
* @date		03. April. 2018
* @author	Renato Dc
*
* All rights reserved.
*/
/* Peripheral group ----------------------------------------------------------- */
/** @addtogroup RIT
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_rit.h"
#include "lpc17xx_clkpwr.h"

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup RIT_Public_Functions
 * @{
 */

/******************************************************************************//*
 * @brief 		Set compare value, mask value and time counter value
 * @param[in]	RITx is RIT peripheral selected, should be: LPC_RIT
 * @param[in]	time_interval: timer interval value (ms)
 * @return 		None
 *******************************************************************************/
void RIT_Init(LPC_RIT_TypeDef *RITx, uint32_t time_interval)
{
	uint32_t clock_rate;

	clock_rate = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_RIT);

	RITx->RICOMPVAL = (clock_rate /1000) * time_interval;

	/* Set timer enable clear bit to clear timer to 0 whenever
	 * counter value equals the contents of RICOMPVAL
	 */
	RITx->RICTRL |= (1<<1);
}


/******************************************************************************//*
 * @brief 		Enable/Disable Timer
 * @param[in]	RITx is RIT peripheral selected, should be: LPC_RIT
 * @param[in]	NewState 	New State of this function
 * 					-ENABLE: Enable Timer
 * 					-DISABLE: Disable Timer
 * @return 		None
 *******************************************************************************/
void RIT_Cmd(LPC_RIT_TypeDef *RITx, FunctionalState NewState)
{
	if(NewState == ENABLE) RITx->RICTRL |= RIT_CTRL_TEN;
	
	else                 	 RITx->RICTRL &= ~RIT_CTRL_TEN;
}

/******************************************************************************//*
 * @brief 		Timer Enable/Disable on debug
 * @param[in]	RITx is RIT peripheral selected, should be: LPC_RIT
 * @param[in]	NewState 	New State of this function
 * 					-ENABLE: The timer is halted whenever a hardware break condition occurs
 * 					-DISABLE: Hardware break has no effect on the timer operation
 * @return 		None
 *******************************************************************************/
void RIT_TimerDebugCmd(LPC_RIT_TypeDef *RITx, FunctionalState NewState)
{
	//Timer Enable/Disable on break
	if(NewState == ENABLE)	RITx->RICTRL |= RIT_CTRL_ENBR;
	
	else									  RITx->RICTRL &= ~RIT_CTRL_ENBR;
	
}
/******************************************************************************//*
 * @brief 		Check whether interrupt flag is set or not
 * @param[in]	RITx is RIT peripheral selected, should be: LPC_RIT
 * @return 		Current interrupt status, could be: SET/RESET
 *******************************************************************************/
IntStatus RIT_GetIntStatus(LPC_RIT_TypeDef *RITx)
{
	IntStatus result;

	if((RITx->RICTRL&RIT_CTRL_INTEN)==1)	result= SET;
	
	else return RESET;
	//clear interrupt flag
	RITx->RICTRL |= RIT_CTRL_INTEN;
	
	return result;
}

/**
 * @}
 */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
