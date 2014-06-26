/**
  ******************************************************************************
  * File Name          : dma.c
  * Date               : 23/06/2014 10:51:07
  * Description        : This file provides code for the configuration
  *                      of all the requested memory to memory DMA transfers.
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
#include "dma.h"

static uint8_t dma1_clock_bits = 0;
static uint8_t dma2_clock_bits = 0;

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();
}

/******************************************************************************
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_TypeDef *) DMA2_Stream7_BASE)
******************************************************************************/

void DMA_Clock_Get(DMA_Stream_TypeDef* stream)
{
	uint8_t* dma = 0;
	int pos = 0;
	
	if (DMA1_Stream0 == stream ||
			DMA1_Stream1 == stream ||
			DMA1_Stream2 == stream ||
			DMA1_Stream3 == stream ||
			DMA1_Stream4 == stream ||
			DMA1_Stream5 == stream ||
			DMA1_Stream6 == stream ||
			DMA1_Stream7 == stream)
	{
		pos = (stream - DMA1_Stream0);
		
		dma = &dma1_clock_bits;
		
		(*dma) |= (1 << pos);
		if (*dma)
		{
			__DMA1_CLK_ENABLE();
		}
		
		return;
	}
	
	if (DMA2_Stream0 == stream ||
			DMA2_Stream1 == stream ||
			DMA2_Stream2 == stream ||
			DMA2_Stream3 == stream ||
			DMA2_Stream4 == stream ||
			DMA2_Stream5 == stream ||
			DMA2_Stream6 == stream ||
			DMA2_Stream7 == stream)
	{
		pos = (stream - DMA2_Stream0);
		
		dma = &dma2_clock_bits;
		
		(*dma) |= (1 << pos);
		if (*dma)
		{
			__DMA2_CLK_ENABLE();
		}
		
		return;		
	}
}

void DMA_Clock_Put(DMA_Stream_TypeDef* stream)
{
	uint8_t* dma = 0;
	int pos = 0;
	
	if (DMA1_Stream0 == stream ||
			DMA1_Stream1 == stream ||
			DMA1_Stream2 == stream ||
			DMA1_Stream3 == stream ||
			DMA1_Stream4 == stream ||
			DMA1_Stream5 == stream ||
			DMA1_Stream6 == stream ||
			DMA1_Stream7 == stream)
	{
		pos = (stream - DMA1_Stream0);
		
		dma = &dma1_clock_bits;
		
		(*dma) &= ~(1 << pos);
		if (*dma == 0)
		{
			__DMA1_CLK_DISABLE();
		}
		
		return;
	}
	
	if (DMA2_Stream0 == stream ||
			DMA2_Stream1 == stream ||
			DMA2_Stream2 == stream ||
			DMA2_Stream3 == stream ||
			DMA2_Stream4 == stream ||
			DMA2_Stream5 == stream ||
			DMA2_Stream6 == stream ||
			DMA2_Stream7 == stream)
	{
		pos = (stream - DMA2_Stream0);
		
		dma = &dma2_clock_bits;
		
		(*dma) &= ~(1 << pos);
		if (*dma == 0)
		{
			__DMA2_CLK_DISABLE();
		}
		
		return;		
	}	
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
