#include "ch32v003fun.h"
#include "funplus.h"
#include "i2c.h"
#include "quaternion_filters.h"
#include "mpu9250.h"

//uint8_t test_data[] = {0x72, 00, 00, 00, 00};
float testResults[6];

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
#if 1
		//i2c_write(0b1101000, test_data, sizeof(test_data));
		uint8_t test_data;

		test_data = i2c_read_reg(0x68, 0x75);
		if (test_data == 0x71) {
			UART_WriteStr("MPU-9250 Found");UART_WriteStr(CRLF);
			GPIO_Clear(GPIOD, 4); // Turn off PD4 (LED ON)
		} else {
			GPIO_Set(GPIOD, 4); // Turn on PD4 (LED OFF)
		}

		test_data = i2c_read_reg(0x76, 0xD0);
		if (test_data == 0x58) {
			UART_WriteStr("BNP280 Found");UART_WriteStr(CRLF);
			GPIO_Clear(GPIOD, 0); // Turn off PD0 (LED ON)
		} else {
			GPIO_Set(GPIOD, 0); // Turn on PD0 (LED OFF)
		}
#endif
		// wake up device
		// Clear sleep mode bit (6), enable all sensors
		i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);
		Delay_Ms(100); // Wait for all registers to reset 

		// get stable time source
		// Auto select clock source to be PLL gyroscope reference if ready else
		i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);
		Delay_Ms(200);
		MPU9250SelfTest(testResults);
		UART_WriteStr("Test Results");UART_WriteStr(CRLF);
        UART_WriteStr("ax = ");UART_WriteFloat(testResults[0], 2);
		UART_WriteStr("    gx = ");UART_WriteFloat(testResults[3], 2);UART_WriteStr(CRLF);
        UART_WriteStr("ay = ");UART_WriteFloat(testResults[1], 2);
		UART_WriteStr("    gy = ");UART_WriteFloat(testResults[4], 2);UART_WriteStr(CRLF);
        UART_WriteStr("az = ");UART_WriteFloat(testResults[2], 2);
		UART_WriteStr("    gz = ");UART_WriteFloat(testResults[5], 2);UART_WriteStr(CRLF);
	}
}
