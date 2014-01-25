/******************************************************************************
 * @file			rc_transmitter.h
 * @author		Skywalker
 * @brief			STM32 user peripheral library for Futaba transmitter
							Decode PPM signal from trainer port
 * @version		V1.0
 * @date			26. Nov 2013
 *
 * @par
 * Include this header in your project source file(s) and set correct include
 * path.
 ******************************************************************************/
 
#ifndef __RC_TRANSMITTER_H
#define __RC_TRANSMITTER_H

#ifdef __cplusplus
// extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------- */
#include "stm32f10x.h"

/* Exported Define ---------------------------------------------------------- */

/* Exported type definitions ------------------------------------------------ */

/** 
  * @brief  Data Status
  */
	
typedef enum
{
	NA = 0,	//Not available
	CAPTURING,
	READY
} RC_Tx_DataStatus;
	
/** 
  * @brief  Transmitter Object definition  
  */

typedef struct
{
	/** @membergroup Configuration variables
  * @{
  */
	uint32_t 				GPIO_CLK;				/*!< Specifies the GPIO peripheral clock 
																	This parameter can be a value of @ref APB2_peripheral*/

	GPIO_TypeDef*		GPIO_Port;			/*!< Specifies the GPIO port to be configured.
                                  This parameter can be any value of GPIOA ~ GPIOG */
	
	uint16_t				GPIO_PIN;				/*!< Specifies the GPIO pin to be configured.
                                  This parameter can be any value of @ref GPIO_pins_define */
	
	uint32_t				TIM_CLK;				/*!< Specifies the TIM peripheral clock 
																	This parameter can be a value of @ref APB1_peripheral*/
	
	TIM_TypeDef*		TIMx;						/*!< Specifies the TIM Peripheral.
                                  This parameter can be a value of TIM2, TIM3, TIM4 or TIM5*/
	
	uint16_t				TIM_Channel;		/*!< Specifies the TIM channel.
                                  This parameter can be a value of @ref TIM_Channel */
	
	uint16_t				TIM_IT;					/*!< Specifies the TIM interrupt source.
                                  This parameter can be TIM_IT_CC1, TIM_IT_CC2, TIM_IT_CC3 or TIM_IT_CC4 */
																	
	IRQn_Type				TIM_IRQn;												/*!< Specifies the IRQ channel to be used.
                                                   This parameter can be a value of @ref IRQn_Type, TIMx_IRQn. */
																										
	uint8_t					IRQChannelPreemptionPriority;  	/*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */
																									 
  uint8_t 				IRQChannelSubPriority;         	/*!< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */
	
	uint16_t				PPM_Polarity;   /*!< Specifies the active edge of the input signal.
                                  This parameter can be a value of @ref TIM_Input_Capture_Polarity */
	
	uint8_t					Max_Channel;		/*!< Specifies 'the number of active edges' - 1 in one signal period
                                  1 ~ 16 integer, please set it correctly */
	/**
  * @}	Configuration variables
  */
	
	/** @membergroup Working variables
  * @{
  */
	uint16_t				PW[16];	/*!< Decoded pulse width of each channel, presented in 1/24 microseconds */
	
	RC_Tx_DataStatus	DataReady;					/*!< For synchronization purpose, DataReady flag is READY when all of the active edges are captured */
	
	uint32_t				CaptureErrorCount;	/*!< If 'the number of active edges' - 1 doesn't match Max_Channel for one
																			time, this variable will plus 1 */
	/**
  * @}	Working variables
  */
}	RC_Tx_Object;

#ifdef __cplusplus
	}
#endif /* __cplusplus */

/* Exported functions ------------------------------------------------------- */
void RC_Tx_Init(RC_Tx_Object* tx);
void RC_Tx_IRQHandler(RC_Tx_Object* tx);
	
#endif
