#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "unity_fixture.h"

/******************************************************************************

analyze the problem

OR-ing Switch Model

1. There should be 8 switches for either DMA1 or DMA2;
2. For either DMA, if 1 or more switch turn on, the clock should be on.
3. For eiterh DMA, if no switch turned on, the clock should be off.

Then Test Plan:
1. clock off, turn A on, clock on.
2. clock on, switch A on, turn A off, clock off.
3. clock on, switch A on, switch B on, turn A off, clock on.

******************************************************************************/

static DMA_Clock_TypeDef dma_clock = {0,0};

static bool dma1_clock_enabled(void)
{
	return (RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN) ? true : false;
}

TEST_GROUP(DMA_Clock);

TEST_SETUP(DMA_Clock)
{
	memset(&dma_clock, 0, sizeof(dma_clock));
	__DMA1_CLK_DISABLE();
	__DMA2_CLK_DISABLE();
}

TEST_TEAR_DOWN(DMA_Clock)
{
	__DMA1_CLK_ENABLE();
	__DMA2_CLK_ENABLE();
}

/******************************************************************************
#define __DMA1_CLK_ENABLE()          (RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN))
#define __DMA2_CLK_ENABLE()          (RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2EN))
******************************************************************************/



TEST(DMA_Clock, ClockOffTurnOneOnClockOn)
{
	// assume all clocks off.
	DMA_Clock_Get(&dma_clock, DMA1_Stream5);
	TEST_ASSERT_TRUE(dma1_clock_enabled());
}

TEST(DMA_Clock, ClockOffTurnOneOnBitSet)
{	
	DMA_Clock_Get(&dma_clock, DMA1_Stream5);
	TEST_ASSERT_TRUE(DMA_Clock_Status(&dma_clock, DMA1_Stream5));
}

TEST(DMA_Clock, ClockOnTurnOffClockOff)
{
	DMA_Clock_Get(&dma_clock, DMA1_Stream5);
	DMA_Clock_Put(&dma_clock, DMA1_Stream5);
	TEST_ASSERT_FALSE(dma1_clock_enabled());
}

TEST(DMA_Clock, ClockOnTurnOffBitClear)
{
	DMA_Clock_Get(&dma_clock, DMA1_Stream5);
	DMA_Clock_Put(&dma_clock, DMA1_Stream5);
	TEST_ASSERT_FALSE(DMA_Clock_Status(&dma_clock, DMA1_Stream5));
}

TEST_GROUP_RUNNER(DMA_Clock)
{
	RUN_TEST_CASE(DMA_Clock, ClockOffTurnOneOnClockOn);
	RUN_TEST_CASE(DMA_Clock, ClockOffTurnOneOnBitSet);
	RUN_TEST_CASE(DMA_Clock, ClockOnTurnOffClockOff);
	RUN_TEST_CASE(DMA_Clock, ClockOnTurnOffBitClear);
}

