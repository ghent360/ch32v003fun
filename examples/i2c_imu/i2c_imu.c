// Could be defined here, or in the processor defines.
#define SYSTEM_CORE_CLOCK 48000000
#define APB_CLOCK SYSTEM_CORE_CLOCK

#include "ch32v003fun.h"
#include "funplus.h"
#include "i2c.h"
#include "quaternion_filters.h"

//uint8_t test_data[] = {0x72, 00, 00, 00, 00};

int main()
{
	// 48MHz internal clock
	SystemInit48HSI();

	// Enable all clocks in one go.
	ABP2ClockEnable(
		RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1);
	ABP1ClockEnable(RCC_APB1Periph_I2C1);

    // Configure each GPIO port in one go.
	GPIO_Configure_Pins(GPIOC,
	  GPIO_Configure_Mask(1) |
	  GPIO_Configure_Mask(2),
	  GPIO_Configure_Mode(1, GPIO_Speed_10MHz, GPIO_CNF_OUT_OD_AF) |
	  GPIO_Configure_Mode(2, GPIO_Speed_10MHz, GPIO_CNF_OUT_OD_AF));

	GPIO_Configure_Pins(GPIOD,
	  GPIO_Configure_Mask(0) |
	  GPIO_Configure_Mask(4) |
	  GPIO_Configure_Mask(5) |
	  GPIO_Configure_Mask(6),
	  GPIO_Configure_Mode(0, GPIO_Speed_10MHz, GPIO_CNF_OUT_PP) |
	  GPIO_Configure_Mode(4, GPIO_Speed_10MHz, GPIO_CNF_OUT_PP) |
	  GPIO_Configure_Mode(5, GPIO_Speed_10MHz, GPIO_CNF_OUT_PP_AF) |
	  GPIO_Configure_Mode(6, GPIO_SPEED_IN, GPIO_CNF_IN_FLOATING));

	// PD5(TX), PD6(RX) - 115200, 8n1
	UART_SetupRxTx();
	i2c_init_master();

	while(1)
	{
		//i2c_write(0b1101000, test_data, sizeof(test_data));
		uint8_t test_data;

		test_data = i2c_read_reg(0x68, 0x75);
		if (test_data == 0x71) {
			UART_WriteStr("MPU-9250 Found\n\r");
			GPIO_Clear(GPIOD, 4); // Turn off PD4 (LED ON)
		} else {
			GPIO_Set(GPIOD, 4); // Turn on PD4 (LED OFF)
		}

		test_data = i2c_read_reg(0x76, 0xD0);
		if (test_data == 0x58) {
			UART_WriteStr("BNP280 Found\n\r");
			GPIO_Clear(GPIOD, 0); // Turn off PD0 (LED ON)
		} else {
			GPIO_Set(GPIOD, 0); // Turn on PD0 (LED OFF)
		}

		Delay_Ms(1000);
	}
}
