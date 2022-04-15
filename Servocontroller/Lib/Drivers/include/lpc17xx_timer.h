/**********************************************************************
* $Id$		lpc17xx_timer.h				2010-05-21
*//**
* @file		lpc17xx_timer.h
* @brief	Contains all macro definitions and function prototypes
* 			support for Timer firmware library on LPC17xx
* @version	2.0
* @date		21. May. 2010
* @author	NXP MCU SW Application Team
*
* Copyright(C) 2010, NXP Semiconductor
* All rights reserved.
*/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup TIM TIM (Timer)
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef __LPC17XX_TIMER_H_
#define __LPC17XX_TIMER_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Private Macros ------------------------------------------------------------- */
/** @defgroup TIM_Private_Macros TIM Private Macros
 * @{
 */

/* --------------------- BIT DEFINITIONS -------------------------------------- */
/**********************************************************************
** Interrupt information
**********************************************************************/

/**********************************************************************
** Timer interrupt register definitions
**********************************************************************/
/** Macro for getting a timer match interrupt bit */
#define TIM_MATCH_INT(n)		(_BIT(n & 0x0F))
/** Macro for getting a capture event interrupt bit */
#define TIM_CAP_INT(n)     (_BIT(((n & 0x0F) + 4)))

/**********************************************************************
* Timer control register definitions
**********************************************************************/
/** Timer/counter enable bit */
#define TIM_ENABLE			((uint32_t)(1<<0))
/** Timer/counter reset bit */
#define TIM_RESET			((uint32_t)(1<<1))
/** Timer control bit mask */
#define TIM_TCR_MASKBIT		((uint32_t)(3))

/**********************************************************************
* Timer match control register definitions
**********************************************************************/
/** Bit location for interrupt on MRx match, n = 0 to 3 */
#define TIM_INT_ON_MATCH(n)      	(_BIT((n * 3)))
/** Bit location for reset on MRx match, n = 0 to 3 */
#define TIM_RESET_ON_MATCH(n)    	(_BIT(((n * 3) + 1)))
/** Bit location for stop on MRx match, n = 0 to 3 */
#define TIM_STOP_ON_MATCH(n)     	(_BIT(((n * 3) + 2)))
/** Timer Match control bit mask */
#define TIM_MCR_MASKBIT			   ((uint32_t)(0x0FFF))
/** Timer Match control bit mask for specific channel*/
#define	TIM_MCR_CHANNEL_MASKBIT(n)		((uint32_t)(7<<(n*3)))

/**********************************************************************
* Timer capture control register definitions
**********************************************************************/
/** Bit location for CAP.n on CRx rising edge, n = 0 to 3 */
#define TIM_CAP_RISING(n)   	(_BIT((n * 3)))
/** Bit location for CAP.n on CRx falling edge, n = 0 to 3 */
#define TIM_CAP_FALLING(n)   	(_BIT(((n * 3) + 1)))
/** Bit location for CAP.n on CRx interrupt enable, n = 0 to 3 */
#define TIM_INT_ON_CAP(n)    	(_BIT(((n * 3) + 2)))
/** Mask bit for rising and falling edge bit */
#define TIM_EDGE_MASK(n)		(_SBF((n * 3), 0x03))
/** Timer capture control bit mask */
#define TIM_CCR_MASKBIT			((uint32_t)(0x3F))
/** Timer Capture control bit mask for specific channel*/
#define	TIM_CCR_CHANNEL_MASKBIT(n)		((uint32_t)(7<<(n*3)))

/**********************************************************************
* Timer external match register definitions
**********************************************************************/
/** Bit location for output state change of MAT.n when external match
   happens, n = 0 to 3 */
#define TIM_EM(n)    			_BIT(n)
/** Output state change of MAT.n when external match happens: no change */
#define TIM_EM_NOTHING    	((uint8_t)(0x0))
/** Output state change of MAT.n when external match happens: low */
#define TIM_EM_LOW         	((uint8_t)(0x1))
/** Output state change of MAT.n when external match happens: high */
#define TIM_EM_HIGH        	((uint8_t)(0x2))
/** Output state change of MAT.n when external match happens: toggle */
#define TIM_EM_TOGGLE      	((uint8_t)(0x3))
/** Macro for setting for the MAT.n change state bits */
#define TIM_EM_SET(n,s) 	(_SBF(((n << 1) + 4), (s & 0x03)))
/** Mask for the MAT.n change state bits */
#define TIM_EM_MASK(n) 		(_SBF(((n << 1) + 4), 0x03))
/** Timer external match bit mask */
#define TIM_EMR_MASKBIT	0x0FFF

/**********************************************************************
* Timer Count Control Register definitions
**********************************************************************/
/** Mask to get the Counter/timer mode bits */
#define TIM_CTCR_MODE_MASK  0x3
/** Mask to get the count input select bits */
#define TIM_CTCR_INPUT_MASK 0xC
/** Timer Count control bit mask */
#define TIM_CTCR_MASKBIT	0xF
#define TIM_COUNTER_MODE ((uint8_t)(1))


/* ---------------- CHECK PARAMETER DEFINITIONS ---------------------------- */
/** Macro to determine if it is valid TIMER peripheral */
#define PARAM_TIMx(n)	((((uint32_t *)n)==((uint32_t *)LPC_TIM0)) || (((uint32_t *)n)==((uint32_t *)LPC_TIM1)) \
|| (((uint32_t *)n)==((uint32_t *)LPC_TIM2)) || (((uint32_t *)n)==((uint32_t *)LPC_TIM3)))

/* Macro check interrupt type */
#define PARAM_TIM_INT_TYPE(TYPE)	((TYPE ==TIM_MR0_INT)||(TYPE ==TIM_MR1_INT)\
||(TYPE ==TIM_MR2_INT)||(TYPE ==TIM_MR3_INT)\
||(TYPE ==TIM_CR0_INT)||(TYPE ==TIM_CR1_INT))

/* Macro check TIMER mode */
#define PARAM_TIM_MODE_OPT(MODE)	((MODE == TIM_TIMER_MODE)||(MODE == TIM_COUNTER_RISING_MODE)\
|| (MODE == TIM_COUNTER_RISING_MODE)||(MODE == TIM_COUNTER_RISING_MODE))

/* Macro check TIMER prescale value */
#define PARAM_TIM_PRESCALE_OPT(OPT)	((OPT == TIM_PRESCALE_TICKVAL)||(OPT == TIM_PRESCALE_USVAL))

/* Macro check TIMER counter intput mode */
#define PARAM_TIM_COUNTER_INPUT_OPT(OPT)	((OPT == TIM_COUNTER_INCAP0)||(OPT == TIM_COUNTER_INCAP1))

/* Macro check TIMER external match mode */
#define PARAM_TIM_EXTMATCH_OPT(OPT)	((OPT == TIM_EXTMATCH_NOTHING)||(OPT == TIM_EXTMATCH_LOW)\
||(OPT == TIM_EXTMATCH_HIGH)||(OPT == TIM_EXTMATCH_TOGGLE))

/* Macro check TIMER external match mode */
#define PARAM_TIM_CAP_MODE_OPT(OPT)	((OPT == TIM_CAPTURE_NONE)||(OPT == TIM_CAPTURE_RISING) \
||(OPT == TIM_CAPTURE_FALLING)||(OPT == TIM_CAPTURE_ANY))

/**
 * @}
 */


/* Public Types --------------------------------------------------------------- */
/** @defgroup TIM_Public_Types TIM Public Types
 * @{
 */

/***********************************************************************
 * Timer device enumeration
**********************************************************************/
/** @brief interrupt type */
typedef enum
{
	TIM_INT_MR0 =1<<0, /*!< interrupt for Match channel 0*/
	TIM_INT_MR1 =1<<1, /*!< interrupt for Match channel 1*/
	TIM_INT_MR2 =1<<2, /*!< interrupt for Match channel 2*/
	TIM_INT_MR3 =1<<3, /*!< interrupt for Match channel 3*/
	TIM_INT_CR0 =1<<4, /*!< interrupt for Capture channel 0*/
	TIM_INT_CR1 =1<<5 /*!< interrupt for Capture channel 1*/
}TIM_INT_E;

/** @brief Timer/counter operating mode */
typedef enum
{
	TIM_MODE_TIMER = 0,				  /*!< Timer mode */
	TIM_MODE_COUNTER_RISING,		/*!< Counter rising mode */
	TIM_MODE_COUNTER_FALLING,		/*!< Counter falling mode */
	TIM_MODE_COUNTER_ANY			  /*!< Counter on both edges */
} TIM_MODE_E;

/** @brief Counter input option */
typedef enum
{
	TIM_COUNTER_INCAP0 = 0,			/*!< CAPn.0 input pin for TIMERn */
	TIM_COUNTER_INCAP1,				/*!< CAPn.1 input pin for TIMERn */
} TIM_COUNTER_INPUT_E;

/** @brief Timer/Counter external match option */
typedef enum
{
	TIM_EXTMATCH_NOTHING = 0,		/*!< Do nothing for external output pin if match */
	TIM_EXTMATCH_LOW,				    /*!< Force external output pin to low if match */
	TIM_EXTMATCH_HIGH,				  /*!< Force external output pin to high if match */
	TIM_EXTMATCH_TOGGLE				  /*!< Toggle external output pin if match */
}TIM_EXTMATCH_E;

/** @brief Configuration structure in TIMER or COUNTER mode */
typedef struct {
	uint32_t 								PrescaleValue;
	TIM_MODE_E						  CounterOption;	
	TIM_COUNTER_INPUT_E 	  CountInputSelect;
} TIM_TIMER_InitTypeDef;

/** @brief Match channel configuration structure */
typedef struct {
	
	uint32_t MatchValue;								/** Match value */
	uint8_t MatchChannel;								/**< Match channel, should be in range		from 0..3 */
	FunctionalState IntOnMatch;					/**< Interrupt On match*/
	FunctionalState StopOnMatch;				/**< Stop On match*/
	FunctionalState ResetOnMatch;				/**< Reset On match*/
	TIM_EXTMATCH_E ExtMatchOutputType;	/**< External Match Output type, should be:
																				 -	 TIM_EXTMATCH_NOTHING:	Do nothing for external output pin if match
																				 -   TIM_EXTMATCH_LOW:	Force external output pin to low if match
																				 - 	 TIM_EXTMATCH_HIGH: Force external output pin to high if match
																				 -   TIM_EXTMATCH_TOGGLE: Toggle external output pin if match.*/
} TIM_MATCH_InitTypeDef;

/** @brief Capture Input configuration structure */
typedef struct {
	TIM_COUNTER_INPUT_E CaptureChannel;	/**< Capture channel: 0,1 */
	FunctionalState RisingEdge;	  			/**< caption rising edge*/
	FunctionalState FallingEdge;				/**< caption falling edge*/
	FunctionalState IntOnCaption;	  		/**< Interrupt On caption*/
} TIM_CAPTURE_InitTypeDef;

/**
 * @}
 */




/* Public Functions ----------------------------------------------------------- */
/** @defgroup TIM_Public_Functions TIM Public Functions
 * @{
 */
/* Init/DeInit TIM functions -----------*/
void TIM_Init(LPC_TIM_TypeDef *TIMx, TIM_TIMER_InitTypeDef *TIM_ConfigStruct);

/* TIM interrupt functions -------------*/
void TIM_ClearIntPending(LPC_TIM_TypeDef *TIMx, TIM_INT_E IntFlag);
void TIM_ClearIntCapturePending(LPC_TIM_TypeDef *TIMx, TIM_INT_E IntFlag);
FlagStatus TIM_GetIntStatus(LPC_TIM_TypeDef *TIMx, TIM_INT_E IntFlag);
FlagStatus TIM_GetIntCaptureStatus(LPC_TIM_TypeDef *TIMx, TIM_INT_E IntFlag);

/* TIM configuration functions --------*/
void TIM_ConfigMatch(LPC_TIM_TypeDef *TIMx, TIM_MATCH_InitTypeDef *TIM_MatchConfigStruct);
void TIM_SetMatchExt(LPC_TIM_TypeDef *TIMx,TIM_EXTMATCH_E ext_match );
void TIM_ConfigCapture(LPC_TIM_TypeDef *TIMx, TIM_CAPTURE_InitTypeDef *TIM_CaptureConfigStruct);

uint32_t TIM_GetCaptureValue(LPC_TIM_TypeDef *TIMx, TIM_COUNTER_INPUT_E CaptureChannel);
void TIM_ResetCounter(LPC_TIM_TypeDef *TIMx);
void TIM_Cmd(LPC_TIM_TypeDef *TIMx, FunctionalState NewState);
void TIM_UpdateMatchValue(LPC_TIM_TypeDef *TIMx, uint8_t channel, uint32_t value);
/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __LPC17XX_TIMER_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
