#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "unity_fixture.h"

/******************************************************************************

IP Blocks

PORT: Clock, Config; No IRQ, No Enable
DMA: Clock, Config; Enable used but not in init; IRQ Handler, Config & Enable
UART: Clock, Config, Enable; IRQ Handler, Config & Enable

HAL Layers

UART: State, Error
DMA: State, Error

Initialization Sequence:
		Actions											Consequence								
		MX_GPIO_Init() 						
[x]		...												PORT Clock Enabled 
		MX_DMA_Init()							
[x]		...												DMA Clock Enabled
[x]		...												DMA IRQ Config & Enabled;
		MX_USART2_UART_Init() 
			HAL_UART_Init() 	
				HAL_UART_MspInit()			
[x]				...										UART Clock Enabled
[x]				HAL_GPIO_Init()				PORT(GPIO) Config
[x]				HAL_DMA_Init()				rx DMA Config, dma State -> READY
[x]				...										uart_rx/dma mutual reference
[x]				HAL_DMA_Init()				tx DMA Config, dma State -> READY
[x]				...										uart_tx/dma mutual reference
[x]				...										UART IRQ Config & Enabled
[x]			UART_SetConfig()				UART Config
[x]			...											uart State -> READY
[x]			...											UART Enabled

Init assertion:									reverted in DeInit?				potential power problem?
	port clock enabled						no												yes
	dma clock enabled							no												yes
	dma irq config & enabled			no												???
	dma handler exist							no												???
	uart clock enabled						yes												yes
	port configured	(not reset)		yes
	dma state -> READY 	x 2				yes
	rx/dma linked				x	2				no
	uart irq config & enabled			yes (config not cleared)
	uart configured								no
	uart state->READY							yes (config not cleared)
	uart enabled									no!
	
*******************************************************************************

Deinitialization Sequence:

		HAL_UART_DeInit()
			HAL_UART_MspDeInit()
[x]			...											UART Clock Disabled.
[x]			HAL_GPIO_DeInit()				PORT GPIO reset
				HAL_DMA_DeInit()				rxdma
[x]				...										Disable DMA, clear config/flags
[x]				...										dma State -> RESET
				HAL_DMA_DeInit()				txdma
[x]				...										Disable DMA, clear config/flags
[x]				...										dma State -> RESET
[x]			...											UART IRQ disabled
[x]		...												UART state -> RESET

Deinit Assertion:

		uart clock disabled.
		port gpio reset.
		rxdma State -> RESET
		txdma State -> RESET
		UART IRQ -> Disabled
		UART State -> RESET
	
******************************************************************************/	


UART_HandleTypeDef huart =
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
			.OverSampling = UART_OVERSAMPLING_16
		}
};

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
bool uart_clock_enabled(USART_TypeDef* uart)
{
	if (uart == USART1)
	{
		return (RCC->AHB2ENR & RCC_APB2ENR_USART1EN) ? true : false;
	}
	else if (uart == USART2)
	{
		return (RCC->AHB3ENR & RCC_APB1ENR_USART2EN) ? true : false;
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
		return (RCC->APB1ENR & RCC_APB1ENR_UART4EN) ? true : false;
	}
	else if (uart == USART6)
	{
		return (RCC->APB2ENR & RCC_APB2ENR_USART6EN) ? true : false;
	}
	else 
	{
		assert_param(false);
	}
}

/** MspInit test group **/

TEST_GROUP(Usart_DMA_MspInit);

TEST_SETUP(Usart_DMA_MspInit)
{
}

TEST_TEAR_DOWN(Usart_DMA_MspInit)
{
}

TEST(Usart_DMA_MspInit, UartClockShouldBeEnabled)
{
	HAL_UART_MspInit(&huart);
	TEST_ASSERT_TRUE(uart_clock_enabled(huart.Instance));
}

TEST_GROUP_RUNNER(Usart_DMA_MspInit)
{
	RUN_TEST_CASE(Usart_DMA_MspInit, UartClockShouldBeEnabled);
}

/*****************************************************************************/

TEST_GROUP(Usart_DMA);

TEST_SETUP(Usart_DMA)
{
}

TEST_TEAR_DOWN(Usart_DMA)
{
}

TEST(Usart_DMA, DoNothing)
{
	TEST_ASSERT_TRUE(true);
}

TEST_GROUP_RUNNER(Usart_DMA)
{
	RUN_TEST_CASE(Usart_DMA, DoNothing);
}



