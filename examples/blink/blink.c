// Could be defined here, or in the processor defines.
#define SYSTEM_CORE_CLOCK 48000000

#include "ch32v003fun.h"
#include <stdio.h>

#define APB_CLOCK SYSTEM_CORE_CLOCK

static inline void GPIO_Configure_Pin(
	GPIO_TypeDef* port, uint8_t pin, uint8_t speed, uint8_t mode) {
	uint32_t port_cfg = port->CFGLR;
	port_cfg &= ~(0x0f<<(4*pin));
	port_cfg |= (speed | mode) << (4*pin);
	port->CFGLR = port_cfg;
}

// the on and off masks indicate which port pins to turn on or off.
// mask value of 0 means don't turn anything on or off. should not 
// duplicate pins in the on and off mask or behavior is undefined.
//
// Mask format: bit 0 - pin 0, bit 1 - pin 1 etc. For example mask
// 0b01001001 - means pins 6, 3 and 0 of the corresponding port.
//
// This works on the pins of a single port only. pins masked with 0 are 
// unchanged. GPIO_Write(x, 0, 0) is a noop.
//
// Make sure to write to output pins only.
//
static inline void GPIO_WriteMask(
	GPIO_TypeDef* port, uint8_t on_mask, uint8_t off_mask) {
	port->BSHR = on_mask | (off_mask << 16);
}

int main()
{
	SystemInit48HSI();

	// Enable GPIOs
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;

	// GPIO D0 Push-Pull
	GPIO_Configure_Pin(GPIOD, 0, GPIO_Speed_10MHz, GPIO_CNF_OUT_PP);

	// GPIO D4 Push-Pull
	GPIO_Configure_Pin(GPIOD, 4, GPIO_Speed_10MHz, GPIO_CNF_OUT_PP);

	// GPIO C0 Push-Pull
	//GPIO_Configure_Pin(GPIOC, 0, GPIO_Speed_10MHz, GPIO_CNF_OUT_PP);

    uint32_t count = 0;
	int32_t inc_dec = 1;
	while(1)
	{
		GPIO_WriteMask(GPIOD, 0b00000001, 0b00010000); // Turn on PD0, off PD4
		//GPIO_WriteMask(GPIOC, 0b00000001, 0); 		   // Turn on PC0
		Delay_Ms( 250 - count );
		GPIO_WriteMask(GPIOD, 0b00010000, 0b00000001); // Turn off PD0, on PD4
		//GPIO_WriteMask(GPIOC, 0, 0b00000001);		   // Off PC0
		Delay_Ms( count );
		count += inc_dec;
		if( count > 250 || count == 0 ) {
			inc_dec = -inc_dec;
		}
	}
	return 0;
}

