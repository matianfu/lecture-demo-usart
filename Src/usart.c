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
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

// static UARTEX_HandleTypeDef huartex2;

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
	UARTEX_HandleTypeDef* huartex = container_of(huart, UARTEX_HandleTypeDef, huart);
	
  if(huart->Instance==USART2)
  {
    /* Peripheral clock enable */
    __USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
//    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
//    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		HAL_GPIO_Init(huartex->gpiox, huartex->gpio_init);

    /* Peripheral DMA init*/
//    hdma_usart2_rx.Instance = DMA1_Stream5;
//    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
//    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
//    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
//    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//    HAL_DMA_Init(&hdma_usart2_rx);
		HAL_DMA_Init(huartex->huart.hdmarx);

//		__HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

//    hdma_usart2_tx.Instance = DMA1_Stream6;
//    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
//    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
//    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
//    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//    HAL_DMA_Init(&hdma_usart2_tx);
		HAL_DMA_Init(huartex->huart.hdmatx);

//		__HAL_LINKDMA(huart,hdmatx,hdma_usart2_tx);

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
//  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0); should be moved elsewhere, globally.
		if (huartex->irq_config) 
		{
			HAL_NVIC_SetPriority(huartex->irq_config->irqn, 
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
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
	
	HAL_UART_Transmit(&huart3, &chr, 1, 10);
	return a;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
