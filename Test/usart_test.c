#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "unity_fixture.h"

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



