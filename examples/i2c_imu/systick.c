#include "ch32v003fun.h"
#include "systick.h"

volatile uint32_t systick_cnt;

/*
 * Start up the SysTick IRQ
 */
void systick_init(void)
{
	/* disable default SysTick behavior */
	SysTick->CTLR = 0;
	
	/* enable the SysTick IRQ */
	NVIC_EnableIRQ(SysTicK_IRQn);
	
	/* Set the tick interval to 1ms for normal op */
	SysTick->CMP = (SYSTEM_CORE_CLOCK/1000)-1;
	
	/* Start at zero */
	SysTick->CNT = 0;
	systick_cnt = 0;
	
	/* Enable SysTick counter, IRQ, HCLK/1 */
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE |
					SYSTICK_CTLR_STCLK;
}

/*
 * SysTick ISR just counts ticks
 * note - the __attribute__((interrupt)) syntax is crucial!
 */
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
	// move the compare further ahead in time.
	// as a warning, if more than this length of time
	// passes before triggering, you may miss your
	// interrupt.
	SysTick->CMP += (SYSTEM_CORE_CLOCK/1000);

	/* clear IRQ */
	SysTick->SR = 0;

	/* update counter */
	systick_cnt++;
}
