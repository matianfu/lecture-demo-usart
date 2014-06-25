/**
  ******************************************************************************
  * File Name          : USART.c
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

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "usart.h"
#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

// static UARTEX_HandleTypeDef huartex2;

/* USER CODE END 0 */

// UART_HandleTypeDef huart2;
// UART_HandleTypeDef huart3;


/** forward declaration **/
static UARTEX_HandleTypeDef huartex2;

static DMA_HandleTypeDef hdma_usart2_rx =
{
	.Instance = DMA1_Stream5,
	.Init = 
	{
		.Channel = DMA_CHANNEL_4,
    .Direction = DMA_PERIPH_TO_MEMORY,
    .PeriphInc = DMA_PINC_DISABLE,
    .MemInc = DMA_MINC_ENABLE,
    .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .MemDataAlignment = DMA_MDATAALIGN_BYTE,
    .Mode = DMA_NORMAL,
    .Priority = DMA_PRIORITY_LOW,
    .FIFOMode = DMA_FIFOMODE_DISABLE,
	},
	.Parent = &huartex2.huart,	/* statically linked */
};

static DMA_HandleTypeDef hdma_usart2_tx = 
{
	.Instance = DMA1_Stream6,
	.Init = 
	{
		.Channel = DMA_CHANNEL_4,
		.Direction = DMA_MEMORY_TO_PERIPH,
		.PeriphInc = DMA_PINC_DISABLE,
		.MemInc = DMA_MINC_ENABLE,
		.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
		.MemDataAlignment = DMA_MDATAALIGN_BYTE,
		.Mode = DMA_NORMAL,
		.Priority = DMA_PRIORITY_LOW,
		.FIFOMode = DMA_FIFOMODE_DISABLE,
	},
	.Parent = &huartex2.huart,	/* statically linked */
};

static GPIO_InitTypeDef gpio_init_usart2_pd5_pd6 =
{
	.Pin = GPIO_PIN_5|GPIO_PIN_6,
	.Mode = GPIO_MODE_AF_PP,
	.Pull = GPIO_NOPULL,
	.Speed = GPIO_SPEED_LOW,
	.Alternate = GPIO_AF7_USART2,
};

static UART_IrqConfig uart2_irq_config =
{
	.irqn = USART2_IRQn,
};

static UARTEX_HandleTypeDef huartex2 = 
{
	.gpiox = GPIOD,
	.gpio_init = &gpio_init_usart2_pd5_pd6,
	.huart = 
	{
		.Instance = USART2,
		.Init = 
		{
			.BaudRate = 115200,
			.WordLength = UART_WORDLENGTH_8B,
			.StopBits = UART_STOPBITS_1,
			.Parity = UART_PARITY_NONE,
			.Mode = UART_MODE_TX_RX,
			.HwFlowCtl = UART_HWCONTROL_NONE,
			.OverSampling = UART_OVERSAMPLING_16,
		},
		.hdmarx = &hdma_usart2_rx,	/** statically linked **/
		.hdmatx = &hdma_usart2_tx,
	},
	
	.irq_config = &uart2_irq_config,
};


static GPIO_InitTypeDef gpio_init_usart3_pd8_pd9 =
{
	.Pin = GPIO_PIN_8|GPIO_PIN_9,
	.Mode = GPIO_MODE_AF_PP,
	.Pull = GPIO_NOPULL,
	.Speed = GPIO_SPEED_LOW,
	.Alternate = GPIO_AF7_USART3,
};	
	
static UARTEX_HandleTypeDef huartex3 = 
{
	.gpiox = GPIOD,
	.gpio_init = &gpio_init_usart3_pd8_pd9,
	.huart = 
	{
		.Instance = USART3,
		.Init = 
		{
			.BaudRate = 115200,
			.WordLength = UART_WORDLENGTH_8B,
			.StopBits = UART_STOPBITS_1,
			.Parity = UART_PARITY_NONE,
			.Mode = UART_MODE_TX_RX,
			.HwFlowCtl = UART_HWCONTROL_NONE,
			.OverSampling = UART_OVERSAMPLING_16,
		},
	},
};


/* USART2 init function */

void MX_USART_UART_Init(UART_HandleTypeDef* huart)
{

  huart->Instance = USART2;
  huart->Init.BaudRate = 115200;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(huart);

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{
  HAL_UART_Init(&huartex3.huart);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
	UARTEX_HandleTypeDef* huartex = container_of(huart, UARTEX_HandleTypeDef, huart);
	
  if(huart->Instance==USART2)
  {
    /* Peripheral clock enable */
    __USART2_CLK_ENABLE();
  
		HAL_GPIO_Init(huartex->gpiox, huartex->gpio_init);

    /* Peripheral DMA init*/
		
		if (huartex->huart.hdmarx)
		{
			HAL_DMA_Init(huartex->huart.hdmarx);
		}

//		__HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

		if (huartex->huart.hdmatx)
		{
			HAL_DMA_Init(huartex->huart.hdmatx);
		}

//		__HAL_LINKDMA(huart,hdmatx,hdma_usart2_tx);

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
//  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0); should be moved elsewhere, globally.
		if (huartex->irq_config) 
		{
			HAL_NVIC_SetPriority(	huartex->irq_config->irqn, 
														huartex->irq_config->preempt_priority,
														huartex->irq_config->sub_priority);					
			HAL_NVIC_EnableIRQ(huartex->irq_config->irqn);
		}
  }
  else if(huart->Instance==USART3)
  {
    /* Peripheral clock enable */
    __USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
//    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
		HAL_GPIO_Init(huartex->gpiox, huartex->gpio_init);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART2)
  {
    /* Peripheral clock disable */
    __USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
  else if(huart->Instance==USART3)
  {
    /* Peripheral clock disable */
    __USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

  }
} 

int unity_output_char(int a) 
{
	uint8_t chr = a;
	
	HAL_UART_Transmit(&huartex3.huart, &chr, 1, 10);
	return a;
}

/******************************************************************************
in rcc
#define __USART1_CLK_ENABLE()  (RCC->APB2ENR |= (RCC_APB2ENR_USART1EN))
#define __USART6_CLK_ENABLE()  (RCC->APB2ENR |= (RCC_APB2ENR_USART6EN))
#define __USART2_CLK_ENABLE()  (RCC->APB1ENR |= (RCC_APB1ENR_USART2EN))

in rcc_ex
#define __USART3_CLK_ENABLE()  (RCC->APB1ENR |= (RCC_APB1ENR_USART3EN))
#define __UART4_CLK_ENABLE()   (RCC->APB1ENR |= (RCC_APB1ENR_UART4EN))
#define __UART5_CLK_ENABLE()   (RCC->APB1ENR |= (RCC_APB1ENR_UART5EN))
******************************************************************************/
void HAL_UART_ClockEnable(USART_TypeDef* uart)
{
	if (uart == USART1)
	{
		__USART1_CLK_ENABLE();
	}
	else if (uart == USART2)
	{
		__USART2_CLK_ENABLE();
	}
	else if (uart == USART3)
	{
		__USART3_CLK_ENABLE();
	}
	else if (uart == UART4)
	{
		__UART4_CLK_ENABLE();
	}
	else if (uart == UART5)
	{
		__UART5_CLK_ENABLE();
	}
	else if (uart == USART6)
	{
		__USART6_CLK_ENABLE();
	}
	else
	{
		assert_param(false);
	}
}

void HAL_UART_ClockDisable(USART_TypeDef* uart)
{
	if (uart == USART1)
	{
		__USART1_CLK_DISABLE();
	}
	else if (uart == USART2)
	{
		__USART2_CLK_DISABLE();
	}
	else if (uart == USART3)
	{
		__USART3_CLK_DISABLE();
	}
	else if (uart == UART4)
	{
		__UART4_CLK_DISABLE();
	}
	else if (uart == UART5)
	{
		__UART5_CLK_DISABLE();
	}
	else if (uart == USART6)
	{
		__USART6_CLK_DISABLE();
	}
	else
	{
		assert_param(false);
	}
}


bool HAL_UART_ClockIsEnabled(USART_TypeDef* uart)
{
	if (uart == USART1)
	{
		return (RCC->APB2ENR & RCC_APB2ENR_USART1EN) ? true : false;
	}
	else if (uart == USART2)
	{
		return (RCC->APB1ENR & RCC_APB1ENR_USART2EN) ? true : false;
	}
	else if (uart == USART3)
	{
		return (RCC->APB1ENR & RCC_APB1ENR_USART3EN) ? true : false;
	}
	else if (uart == UART4)
	{
		return (RCC->APB1ENR & RCC_APB1ENR_UART4EN) ? true : false;
	}
	else if (uart == UART5)
	{
		return (RCC->APB1ENR & RCC_APB1ENR_UART5EN) ? true : false;
	}
	else if (uart == USART6)
	{
		return (RCC->APB2ENR & RCC_APB2ENR_USART6EN) ? true : false;
	}
	else 
	{
		assert_param(false);
	}
	
	return false;
}

/**
* @brief This function handles DMA1 Stream6 global interrupt.
*/
void DMA1_Stream6_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(DMA1_Stream6_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}


/**
* @brief This function handles DMA1 Stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  HAL_UART_IRQHandler(&huartex2.huart);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
