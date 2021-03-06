#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "unity_fixture.h"
#include "usart.h"

//	.gpiox = GPIOD,														// r
//	.gpio_init = &gpio_init_usart2_pd5_pd6,		// r
//	.huart = 																	// r/w, conf
//	{
//		.Instance = USART2,
//		.Init = 
//		{
//			.BaudRate = 115200,
//			.WordLength = UART_WORDLENGTH_8B,
//			.StopBits = UART_STOPBITS_1,
//			.Parity = UART_PARITY_NONE,
//			.Mode = UART_MODE_TX_RX,
//			.HwFlowCtl = UART_HWCONTROL_NONE,
//			.OverSampling = UART_OVERSAMPLING_16,
//		},
//		.hdmatx = &usart2_tx_dma_handle,				//	r/w
//		.hdmarx = &usart2_rx_dma_handle,				//	r/w
//	},
//	
//	.dmarx_irq_config = &dmarx_irq_config,		//	r, conf
//	.dmatx_irq_config = &dmatx_irq_config,		// 	r, conf
//	.uart_irq_config = &uart2_irq_config,			// 	r, conf
//	.dma_clock = &DMA_Clock_Singleton,				//	r

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

// extern DMA_HandleTypeDef hdma_usart2_rx;
// extern DMA_HandleTypeDef hdma_usart2_tx;

/** forward declaration **/
static UARTEX_HandleTypeDef huartex2;

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
	},
	.Parent = &huartex2.huart,	/* statically linked */
};

static IRQ_ConfigTypeDef dmarx_irq_config = 
{
	.irqn = DMA1_Stream5_IRQn,
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
	},
	.Parent = &huartex2.huart,	/* statically linked */
};

static IRQ_ConfigTypeDef dmatx_irq_config =
{
	.irqn = DMA1_Stream6_IRQn,
};

static GPIO_InitTypeDef gpio_init_usart2_pd5_pd6 =
{
	.Pin = GPIO_PIN_5|GPIO_PIN_6,
	.Mode = GPIO_MODE_AF_PP,
	.Pull = GPIO_NOPULL,
	.Speed = GPIO_SPEED_LOW,
	.Alternate = GPIO_AF7_USART2,
};

static IRQ_ConfigTypeDef uart2_irq_config =
{
	.irqn = USART2_IRQn,
};


	

static UARTEX_HandleTypeDef huartex2_pd5_pd6 = 
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
		.hdmatx = &usart2_tx_dma_handle,	/** statically linked **/
		.hdmarx = &usart2_rx_dma_handle,
	},
	
	.dmarx_irq_config = &dmarx_irq_config,
	.dmatx_irq_config = &dmatx_irq_config,
	.uart_irq_config = &uart2_irq_config,
	.dma_clock = &DMA_Clock_Singleton,
};

static UART_HandleTypeDef* huart = &huartex2.huart;

bool gpio_modes_all_noninput(GPIO_TypeDef* gpiox, uint32_t pins)
{
	uint32_t pos; /**  mask; **/
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

/**
//TEST(Usart_DMA_MspInit, UartClockShouldBeEnabled)
//{
//	HAL_UART_ClockDisable(huart->Instance);
//	HAL_UART_MspInit(huart);
//	TEST_ASSERT_TRUE(HAL_UART_ClockIsEnabled(huart->Instance));
//}
**/

TEST(Usart_DMA_MspInit, UartClockShouldBeEnabled)
{
	UART_HandleTypeDef* h = &huartex2_pd5_pd6.huart;
	HAL_UART_ClockDisable(h->Instance);
	HAL_UART_MspInit(h);
	TEST_ASSERT_TRUE(HAL_UART_ClockIsEnabled(h->Instance));
}

/** don't use the misleading name, such as "GpioShouldeBeConfigured", since we do NOT fully test the gpio configuration. **/
/** we don't do fully test because we trust the code, we just make sure the function is called. **/
TEST(Usart_DMA_MspInit, GpioShouldBeNonInput)
{
	uint32_t pins = GPIO_PIN_5|GPIO_PIN_6;
	UART_HandleTypeDef* h = &huartex2_pd5_pd6.huart;
	HAL_GPIO_DeInit(GPIOD, pins);
	HAL_UART_MspInit(h);
	TEST_ASSERT_TRUE(gpio_modes_all_noninput(GPIOD, pins));
}

/** without this test, the HAL_DMA_Init function succeed anyway even if clock not enabled. **/
TEST(Usart_DMA_MspInit, RxDMAClockBitShouldBeSet)
{
	DMA_Clock_TypeDef dma_clock, *original;
	memset(&dma_clock, 0, sizeof(dma_clock));
	
	original = huartex2.dma_clock;
	huartex2.dma_clock = &dma_clock;
	
	HAL_UART_MspInit(&huartex2.huart);
	
	TEST_ASSERT_TRUE(DMA_Clock_Status(&dma_clock, huartex2.huart.hdmarx->Instance));

	huartex2.dma_clock = original;
}


TEST(Usart_DMA_MspInit, RxDMAShouldBeInitialized)
{
	HAL_DMA_DeInit(&usart2_rx_dma_handle);
	usart2_rx_dma_handle.State = HAL_DMA_STATE_RESET;
	HAL_UART_MspInit(huart);
	TEST_ASSERT_EQUAL(HAL_DMA_STATE_READY, usart2_rx_dma_handle.State);
}

TEST(Usart_DMA_MspInit, RxDMAIRQShouldBeEnabled)
{
	HAL_NVIC_DisableIRQ(huartex2.dmarx_irq_config->irqn);
	HAL_UART_MspInit(&huartex2.huart);
	TEST_ASSERT_TRUE(irq_enabled(huartex2.dmarx_irq_config->irqn));
}

/** this case is obsolete 
//TEST(Usart_DMA_MspInit, RxDMAShouldBeLinked)
//{
//	HAL_DMA_DeInit(&usart2_rx_dma_handle);
//	usart2_rx_dma_handle.State = HAL_DMA_STATE_RESET;
//	usart2_rx_dma_handle.Parent = 0;

//	HAL_UART_MspInit(huart);
//	
//	TEST_ASSERT_NOT_NULL(huart->hdmarx);
//	TEST_ASSERT_EQUAL_HEX32(&usart2_rx_dma_handle, huart->hdmarx);
//	
//	TEST_ASSERT_NOT_NULL(huart->hdmarx->Parent);
//	TEST_ASSERT_EQUAL_HEX32(huart, huart->hdmarx->Parent);
//}
**/

/** without this test, the HAL_DMA_Init function succeed anyway even if clock not enabled. **/
TEST(Usart_DMA_MspInit, TxDMAClockBitShouldBeSet)
{
	DMA_Clock_TypeDef dma_clock, *original;
	memset(&dma_clock, 0, sizeof(dma_clock));
	
	original = huartex2.dma_clock;
	huartex2.dma_clock = &dma_clock;
	
	HAL_UART_MspInit(&huartex2.huart);
	
	TEST_ASSERT_TRUE(DMA_Clock_Status(&dma_clock, huartex2.huart.hdmatx->Instance));

	huartex2.dma_clock = original;
}

TEST(Usart_DMA_MspInit, TxDMAShouldBeInitialized)
{
	HAL_DMA_DeInit(&usart2_tx_dma_handle);
	usart2_tx_dma_handle.State = HAL_DMA_STATE_RESET;
	HAL_UART_MspInit(huart);
	TEST_ASSERT_EQUAL(HAL_DMA_STATE_READY, usart2_tx_dma_handle.State);
}

TEST(Usart_DMA_MspInit, TxDMAIRQShouldBeEnabled)
{
	HAL_NVIC_DisableIRQ(huartex2.dmatx_irq_config->irqn);
	HAL_UART_MspInit(&huartex2.huart);
	TEST_ASSERT_TRUE(irq_enabled(huartex2.dmatx_irq_config->irqn));
}

/** obsolete case 
//TEST(Usart_DMA_MspInit, TxDMAShouldBeLinked)
//{
//	HAL_DMA_DeInit(&usart2_tx_dma_handle);
//	hdma_usart2_tx.State = HAL_DMA_STATE_RESET;
//	hdma_usart2_tx.Parent = 0;
//	huart->hdmatx = 0;
//	HAL_UART_MspInit(huart);
//	
//	TEST_ASSERT_NOT_NULL(huart->hdmatx);
//	TEST_ASSERT_EQUAL_HEX32(&hdma_usart2_tx, huart->hdmatx);
//	
//	TEST_ASSERT_NOT_NULL(huart->hdmatx->Parent);
//	TEST_ASSERT_EQUAL_HEX32(huart, huart->hdmatx->Parent);
//}
**/

TEST(Usart_DMA_MspInit, UARTIRQEnabled)
{
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_UART_MspInit(huart);
	TEST_ASSERT_TRUE(irq_enabled(USART2_IRQn));
}

TEST_GROUP_RUNNER(Usart_DMA_MspInit)
{
	RUN_TEST_CASE(Usart_DMA_MspInit, UartClockShouldBeEnabled);
	RUN_TEST_CASE(Usart_DMA_MspInit, GpioShouldBeNonInput);
	RUN_TEST_CASE(Usart_DMA_MspInit, RxDMAClockBitShouldBeSet);
	RUN_TEST_CASE(Usart_DMA_MspInit, RxDMAShouldBeInitialized);
//	RUN_TEST_CASE(Usart_DMA_MspInit, RxDMAShouldBeLinked);
	RUN_TEST_CASE(Usart_DMA_MspInit, RxDMAIRQShouldBeEnabled);
	RUN_TEST_CASE(Usart_DMA_MspInit, TxDMAClockBitShouldBeSet);
	RUN_TEST_CASE(Usart_DMA_MspInit, TxDMAShouldBeInitialized);
//	RUN_TEST_CASE(Usart_DMA_MspInit, TxDMAShouldBeLinked);
	RUN_TEST_CASE(Usart_DMA_MspInit, TxDMAIRQShouldBeEnabled);
	RUN_TEST_CASE(Usart_DMA_MspInit, UARTIRQEnabled);
}

/*****************************************************************************/

TEST_GROUP(Usart_DMA);

TEST_SETUP(Usart_DMA)
{
}

TEST_TEAR_DOWN(Usart_DMA)
{
}

TEST(Usart_DMA, UartClockDisable)
{
	HAL_UART_ClockEnable(USART2);
	HAL_UART_ClockDisable(USART2);
	TEST_ASSERT_FALSE(HAL_UART_ClockIsEnabled(USART2));
}

TEST(Usart_DMA, UartClockEnable)
{
	HAL_UART_ClockDisable(USART2);
	HAL_UART_ClockEnable(USART2);
	TEST_ASSERT_TRUE(HAL_UART_ClockIsEnabled(USART2));
}

// extern UART_HandleTypeDef huart2;
	
//TEST(Usart_DMA, MX_USART_UART_Init)
//{
//	UART_HandleTypeDef* h = &huartex2_pd5_pd6.huart;
//	h->State = HAL_UART_STATE_RESET;
//	MX_USART_UART_Init(h);
//	TEST_ASSERT_EQUAL_HEX8(HAL_UART_STATE_READY, h->State);
//}


TEST(Usart_DMA, HAL_UART_Init_TxRx)
{
	uint8_t recv, token = '&';
	UART_HandleTypeDef* h = &huartex2_pd5_pd6.huart;
	
	// MX_USART_UART_Init(h);
	HAL_UART_Init(h);
	
	HAL_UART_Transmit(h, &token, 1, 10);
	while(__HAL_UART_GET_FLAG(h, UART_FLAG_RXNE) == RESET);
	recv = (uint8_t)(h->Instance->DR & (uint16_t)0x00FF);
	TEST_ASSERT_EQUAL_UINT8(token, recv);
}


//TEST(Usart_DMA, DoNothing)
//{
//	TEST_ASSERT_TRUE(true);
//}

TEST_GROUP_RUNNER(Usart_DMA)
{
	// RUN_TEST_CASE(Usart_DMA, DoNothing);
	RUN_TEST_CASE(Usart_DMA, UartClockDisable);
	RUN_TEST_CASE(Usart_DMA, UartClockEnable);
	// RUN_TEST_CASE(Usart_DMA, MX_USART_UART_Init);
	RUN_TEST_CASE(Usart_DMA, HAL_UART_Init_TxRx);
}



