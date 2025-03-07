// Really basic self-contained demo for the ch32v003
// Doesn't rely on any of the weird HAL stuff from CH
// Final executable is ~1/4th the size.

// Could be defined here, or in the processor defines.
#define SYSTEM_CORE_CLOCK 48000000
#define APB_CLOCK SYSTEM_CORE_CLOCK

#include "ch32v003fun.h"
#include <stdio.h>

uint32_t count;

int main()
{
	SystemInit48HSI();
	SetupUART( UART_BRR );

	// Enable GPIOD.
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;

	// GPIO D0 Push-Pull, 10MHz Output
	GPIOD->CFGLR &= ~(0xf<<(4*0));
	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

	while(1)
	{
		GPIOD->BSHR = 1;	 // Turn on GPIOD0
		Delay_Ms( 50 );
		GPIOD->BSHR = 1<<16; // Turn off GPIOD0
		Delay_Ms( 50 );
		printf( "Count: %lu\n", count++ );
	}
}
