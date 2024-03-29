/**********************************************************************
* $Id$		lpc17xx_rit.h				2018-04-03
*//**
* @file		lpc17xx_rit.h
* @brief	Contains all macro definitions and function prototypes
* 			support for RIT firmware library on LPC17xx
* @version	2.0
* @date		03. April. 2018
* @author	Renato Dc
*
* Copyright(C) 2010, NXP Semiconductor
* All rights reserved.
*/
/* Peripheral group ----------------------------------------------------------- */
/** @defgroup RIT RIT (Repetitive Interrupt Timer)
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_RIT_H_
#define LPC17XX_RIT_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif


/* Private Macros ------------------------------------------------------------- */
/** @defgroup RIT_Private_Macros RIT Private Macros
 * @{
 */

/* --------------------- BIT DEFINITIONS -------------------------------------- */
/*********************************************************************//**
 * Macro defines for RIT control register
 **********************************************************************/
/**	Set interrupt flag when the counter value equals the masked compare value */
#define RIT_CTRL_INTEN	((uint32_t) (1))
/** Set timer enable clear to 0 when the counter value equals the masked compare value  */
#define RIT_CTRL_ENCLR 	((uint32_t) _BIT(1))
/** Set timer enable on debug */
#define RIT_CTRL_ENBR	((uint32_t) _BIT(2))
/** Set timer enable */
#define RIT_CTRL_TEN	((uint32_t) _BIT(3))


/* Public Functions ----------------------------------------------------------- */
/** @defgroup RIT_Public_Functions RIT Public Functions
 * @{
 */
/* RIT config timer functions */
void RIT_Init(LPC_RIT_TypeDef *RITx, uint32_t time_interval);

/* Enable/Disable RIT functions */
void RIT_TimerClearCmd(LPC_RIT_TypeDef *RITx, FunctionalState NewState);
void RIT_Cmd(LPC_RIT_TypeDef *RITx, FunctionalState NewState);
void RIT_TimerDebugCmd(LPC_RIT_TypeDef *RITx, FunctionalState NewState);

/* RIT Interrupt functions */
IntStatus RIT_GetIntStatus(LPC_RIT_TypeDef *RITx);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_RIT_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
