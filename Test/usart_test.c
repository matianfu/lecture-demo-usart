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



