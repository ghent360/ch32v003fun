// Could be defined here, or in the processor defines.
#define SYSTEM_CORE_CLOCK 48000000
#define APB_CLOCK SYSTEM_CORE_CLOCK

#include "ch32v003fun.h"
#include "funplus.h"
#include "i2c.h"
#include "quaternion_filters.h"

//uint8_t test_data[] = {0x72, 00, 00, 00, 00};

#define MPU_9250_ADDR 0b1101000

#define PWR_MGMNT_1_  0x6B
#define CLKSEL_PLL_   0x01
#define H_RESET_      0x80

int main()
{
	// 48MHz internal clock
	SystemInit48HSI();

	// PD5(TX), PD6(RX) - 115200, 8, 1, n
	UART_SetupRxTx(UART_BRR);
	i2c_init_master();
#if 1
	// Done in UART_SetupRxTx
	//ABP2ClockEnable(RCC_APB2Periph_GPIOD);
	GPIO_Configure_Pin(GPIOD, 0, GPIO_Speed_10MHz, GPIO_CNF_OUT_PP);
	GPIO_Configure_Pin(GPIOD, 4, GPIO_Speed_10MHz, GPIO_CNF_OUT_PP);
#endif

#if 0
    /* Select clock source to gyro */
    i2c_write_reg(MPU_9250_ADDR, PWR_MGMNT_1_, CLKSEL_PLL_);
	/* Reset the MPU-9250 */
    i2c_write_reg(MPU_9250_ADDR, PWR_MGMNT_1_, H_RESET_);
    /* Wait for MPU-9250 to come back up */
    Delay_Ms(1000);
    /* Select clock source to gyro */
    i2c_write_reg(MPU_9250_ADDR, PWR_MGMNT_1_, CLKSEL_PLL_);
	Delay_Ms(1000);
#endif
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

        for(test_data = 0; test_data < 100; test_data++)
		{
			if (!UART_IsDataAvailable()) {
				Delay_Ms(10);
			} else {
				UART_WriteByte(UART_ReadByte());
			}
		}
	}
}
