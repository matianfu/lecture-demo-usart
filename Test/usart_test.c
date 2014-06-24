#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "unity_fixture.h"
#include "usart.h"

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

static UART_HandleTypeDef uart2 =
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
		},
};

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

static DMA_HandleTypeDef usart2_rx_dma_handle =
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
	}
};

static DMA_HandleTypeDef usart2_tx_dma_handle = 
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
			}
};

static GPIO_InitTypeDef gpio_init_usart2_pd5_pd6 =
{
	.Pin = GPIO_PIN_5|GPIO_PIN_6,
	.Mode = GPIO_MODE_AF_PP,
	.Pull = GPIO_NOPULL,
	.Speed = GPIO_SPEED_LOW,
	.Alternate = GPIO_AF7_USART2,
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
	},
};

static UART_HandleTypeDef* huart = &uart2;

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
void uart_clock_disable(USART_TypeDef* uart)
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

bool uart_clock_enabled(USART_TypeDef* uart)
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

bool gpio_modes_all_noninput(GPIO_TypeDef* gpiox, uint32_t pins)
{
	uint32_t pos, mask;
	for (pos = 0; pos < 16; pos++)
	{
		if (((uint32_t)1 << pos) & pins)
		{
			if ((gpiox->MODER & (GPIO_MODER_MODER0 << (pos * 2))) == 0)
				return false;
		}
	}
	return true;
}

bool irq_enabled(IRQn_Type IRQn) 
{
	uint32_t reg;
	
	/** or NVIC->ISER[(uint32_t)((int32_t)IRQn) >> 5]; **/
	reg = NVIC->ISER[(uint32_t)((int32_t)IRQn) >> 5];
	return (reg & (1 << ((uint32_t)(IRQn) & 0x1F))) ? true : false;
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
	uart_clock_disable(huart->Instance);
	HAL_UART_MspInit(huart);
	TEST_ASSERT_TRUE(uart_clock_enabled(huart->Instance));
}

/** don't use the misleading name, such as "GpioShouldeBeConfigured", since we do NOT fully test the gpio configuration. **/
/** we don't do fully test because we trust the code, we just make sure the function is called. **/
TEST(Usart_DMA_MspInit, GpioShouldBeNonInput)
{
	uint32_t pins = GPIO_PIN_5|GPIO_PIN_6;
	HAL_GPIO_DeInit(GPIOD, pins);
	HAL_UART_MspInit(huart);
	TEST_ASSERT_TRUE(gpio_modes_all_noninput(GPIOD, pins));
}

TEST(Usart_DMA_MspInit, RxDMAShouldBeInitialized)
{
	HAL_DMA_DeInit(&usart2_rx_dma_handle);
	hdma_usart2_rx.State = HAL_DMA_STATE_RESET;
	HAL_UART_MspInit(huart);
	TEST_ASSERT_EQUAL(HAL_DMA_STATE_READY, hdma_usart2_rx.State);
}

TEST(Usart_DMA_MspInit, RxDMAShouldBeLinked)
{
	HAL_DMA_DeInit(&usart2_rx_dma_handle);
	hdma_usart2_rx.State = HAL_DMA_STATE_RESET;
	hdma_usart2_rx.Parent = 0;
	huart->hdmarx = 0;
	HAL_UART_MspInit(huart);
	
	TEST_ASSERT_NOT_NULL(huart->hdmarx);
	TEST_ASSERT_EQUAL_HEX32(&hdma_usart2_rx, huart->hdmarx);
	
	TEST_ASSERT_NOT_NULL(huart->hdmarx->Parent);
	TEST_ASSERT_EQUAL_HEX32(huart, huart->hdmarx->Parent);
}

TEST(Usart_DMA_MspInit, TxDMAShouldBeInitialized)
{
	HAL_DMA_DeInit(&usart2_tx_dma_handle);
	hdma_usart2_tx.State = HAL_DMA_STATE_RESET;
	HAL_UART_MspInit(huart);
	TEST_ASSERT_EQUAL(HAL_DMA_STATE_READY, hdma_usart2_tx.State);
}

TEST(Usart_DMA_MspInit, TxDMAShouldBeLinked)
{
	HAL_DMA_DeInit(&usart2_tx_dma_handle);
	hdma_usart2_tx.State = HAL_DMA_STATE_RESET;
	hdma_usart2_tx.Parent = 0;
	huart->hdmatx = 0;
	HAL_UART_MspInit(huart);
	
	TEST_ASSERT_NOT_NULL(huart->hdmatx);
	TEST_ASSERT_EQUAL_HEX32(&hdma_usart2_tx, huart->hdmatx);
	
	TEST_ASSERT_NOT_NULL(huart->hdmatx->Parent);
	TEST_ASSERT_EQUAL_HEX32(huart, huart->hdmatx->Parent);
}

TEST(Usart_DMA_MspInit, IRQEnabled)
{
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_UART_MspInit(huart);
	TEST_ASSERT_TRUE(irq_enabled(USART2_IRQn));
}

TEST_GROUP_RUNNER(Usart_DMA_MspInit)
{
	RUN_TEST_CASE(Usart_DMA_MspInit, UartClockShouldBeEnabled);
	RUN_TEST_CASE(Usart_DMA_MspInit, GpioShouldBeNonInput);
	RUN_TEST_CASE(Usart_DMA_MspInit, RxDMAShouldBeInitialized);
	RUN_TEST_CASE(Usart_DMA_MspInit, RxDMAShouldBeLinked);
	RUN_TEST_CASE(Usart_DMA_MspInit, TxDMAShouldBeInitialized);
	RUN_TEST_CASE(Usart_DMA_MspInit, TxDMAShouldBeLinked);
	RUN_TEST_CASE(Usart_DMA_MspInit, IRQEnabled);
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



