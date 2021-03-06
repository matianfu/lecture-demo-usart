/**
  ******************************************************************************
  * File Name          : USART.h
  * Date               : 23/06/2014 10:51:07
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "dma.h"
	 
typedef struct
{
	IRQn_Type irqn;
	uint32_t preempt_priority;
	uint32_t sub_priority;
} IRQ_ConfigTypeDef;	
	 
typedef struct 
{
	GPIO_TypeDef* 				gpiox;
	GPIO_InitTypeDef*			gpio_init;
	
	UART_HandleTypeDef 		huart;
	
	IRQ_ConfigTypeDef*		dmarx_irq_config;
	IRQ_ConfigTypeDef*		dmatx_irq_config;
	IRQ_ConfigTypeDef*		uart_irq_config;
	
	DMA_Clock_TypeDef*		dma_clock;
	
} UARTEX_HandleTypeDef;



void MX_USART_UART_Init(UART_HandleTypeDef* huart);
void MX_USART3_UART_Init(void);

/** utility **/
void HAL_UART_ClockEnable(USART_TypeDef* uart);
void HAL_UART_ClockDisable(USART_TypeDef* uart);
bool HAL_UART_ClockIsEnabled(USART_TypeDef* uart);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
