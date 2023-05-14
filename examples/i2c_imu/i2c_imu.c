#include "ch32v003fun.h"
#include "funplus.h"
#include "i2c.h"
#include "quaternion_filters.h"
#include "mpu9250.h"
#include "systick.h"

//uint8_t test_data[] = {0x72, 00, 00, 00, 00};
//float testResults[6];
#define PI 3.14159265358979323846f
#define DEGREE_TO_RAD (PI / 180.0f)

int main()
{
	// 48MHz internal clock
	SystemInit48HSI();
	systick_init();

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
	GPIO_Set(GPIOD, 4); // Turn PD4 LED OFF
	GPIO_Set(GPIOD, 0); // Turn PD4 LED OFF

	// PD5(TX), PD6(RX) - 115200, 8n1
	UART_SetupRxTx();
	i2c_init_master();

	uint8_t data;

	i2c_read_reg(MPU9250_ADDRESS, MPU9250_WHO_AM_I, &data);
	if (data != 0x71) {
		while(1) {
			GPIO_Clear(GPIOD, 4); // Turn PD4 LED ON
			Delay_Ms(250);
			GPIO_Set(GPIOD, 4); // Turn PD4 LED OFF
			Delay_Ms(250);
		}
	}
	MPU9250_initRes();
	// Calibrate gyro and accelerometers, load biases in bias registers
	MPU9250_calibrate_accelgyro();
	MPU9250_init();

	i2c_read_reg(AK8963_ADDRESS, WHO_AM_I_AK8963, &data);
	if (data != 0x48) {
		while(1) {
			GPIO_Clear(GPIOD, 4); // Turn PD4 LED ON
			Delay_Ms(400);
			GPIO_Set(GPIOD, 4); // Turn PD4 LED OFF
			Delay_Ms(100);
		}
	}
	AK8963_init();
	MPU9250_calibrate_mag();
	uint32_t now;
	uint32_t last_update = systick_cnt;
	while(1)
	{
#if 0
		// wake up device
		// Clear sleep mode bit (6), enable all sensors
		i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80);
		Delay_Ms(20); // Wait for all registers to reset 
		// Set PLL clock source
		i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);
		Delay_Ms(100);
		MPU9250_self_test(testResults);
		UART_WriteStr("Test Results");UART_WriteStr(CRLF);
        UART_WriteStr("ax = ");UART_WriteFloat(testResults[0], 2);
		UART_WriteStr("    gx = ");UART_WriteFloat(testResults[3], 2);UART_WriteStr(CRLF);
        UART_WriteStr("ay = ");UART_WriteFloat(testResults[1], 2);
		UART_WriteStr("    gy = ");UART_WriteFloat(testResults[4], 2);UART_WriteStr(CRLF);
        UART_WriteStr("az = ");UART_WriteFloat(testResults[2], 2);
		UART_WriteStr("    gz = ");UART_WriteFloat(testResults[5], 2);UART_WriteStr(CRLF);
#endif
        if (read_imu_data() == 0) {
			now = systick_cnt;
			// set integration time by time elapsed since last filter update
			deltat = ((now - last_update) / 1000000.0f);
			last_update = now;
			MadgwickQuaternionUpdate(
				-ax, ay, az, 
				gx * DEGREE_TO_RAD, -gy * DEGREE_TO_RAD, -gz * DEGREE_TO_RAD,
				my,  -mx, mz);
		}
	}
}
