#include <math.h>
#include "ch32v003fun.h"
#include "funplus.h"
#include "i2c.h"
#include "quaternion_filters.h"
#include "mpu9250.h"
#include "systick.h"

//uint8_t test_data[] = {0x72, 00, 00, 00, 00};
float testResults[6];
#define PI 3.14159265358979323846f
#define DEGREE_TO_RAD (PI / 180.0f)

uint8_t use_mahony = 0;
float pitch, roll, yaw;
float lin_ax, lin_ay, lin_az;
float a31,a32,a33;

static void calculate_tb_angles() {
	// Define output variables from updated quaternion---these are Tait-Bryan
	// angles, commonly used in aircraft orientation. In this coordinate system,
	// the positive z-axis is down toward Earth.
	//
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true
	// North if corrected for local declination, looking down on the sensor 
	// positive yaw is counterclockwise.
	//
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the
	// Earth is positive, up toward the sky is negative.
	//
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is
	// positive roll.
	//
	// These arise from the definition of the homogeneous rotation matrix
	// constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is,
	// the get the correct orientation the rotations must be applied in the
	// correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	// which has additional links.
	//
	// Software AHRS:
	//   yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	//   pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
	//   roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	//   pitch *= 180.0f / PI;
	//   yaw   *= 180.0f / PI; 
	//   yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	//   if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	//   roll  *= 180.0f / PI;
	float a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
	float a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
	a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
	a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	pitch = -asinf(a32);
	roll  = atan2f(a31, a33);
	yaw   = atan2f(a12, a22);
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI; 
	yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	roll  *= 180.0f / PI;
	lin_ax = ax + a31;
	lin_ay = ay + a32;
	lin_az = az - a33;
}

static void panic(uint16_t d1, uint16_t d2) {
	while(1) {
		GPIO_Clear(GPIOD, 4); // Turn PD4 LED ON
		Delay_Ms(d1);
		GPIO_Set(GPIOD, 4);   // Turn PD4 LED OFF
		Delay_Ms(d2);
	}
}

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
		panic(250, 250);
	}
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
	MPU9250_initRes();
	// Calibrate gyro and accelerometers, load biases in bias registers
	MPU9250_calibrate_accelgyro();
	MPU9250_init();

    // We can communicate with the AK8963 only after the MPU9250 has been
	// initialized.
	i2c_read_reg(AK8963_ADDRESS, WHO_AM_I_AK8963, &data);
	if (data != 0x48) {
		panic(400, 100);
	}
	AK8963_init();
	//MPU9250_calibrate_mag();
	uint32_t now;
	uint32_t last_imu_update = systick_cnt;
	float elapsed_time = 0;
	uint32_t update_cnt = 0;
	uint32_t last_report_time = last_imu_update;
	while(1)
	{
        if (read_imu_data() == 0) {
			now = systick_cnt;
			// set integration time by time elapsed since last filter update
			deltat = ((now - last_imu_update) / 1000.0f);
			elapsed_time += deltat;
			update_cnt++;
			last_imu_update = now;
#if 0			
			if (use_mahony) {
				MahonyQuaternionUpdate(
					-ax, ay, az, 
					gx * DEGREE_TO_RAD, -gy * DEGREE_TO_RAD, -gz * DEGREE_TO_RAD,
					my,  -mx, mz);
			} else {
				MadgwickQuaternionUpdate(
					-ax, ay, az, 
					gx * DEGREE_TO_RAD, -gy * DEGREE_TO_RAD, -gz * DEGREE_TO_RAD,
					my,  -mx, mz);
			}
#endif
		}
		now = systick_cnt;
		if (now - last_report_time > 500) {
			UART_WriteStr("ax, ay, az: ");
			UART_WriteFloat(ax, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(ay, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(az, 2);
			UART_WriteStr(CRLF);
			UART_WriteStr("gx, gy, gz: ");
			UART_WriteFloat(gx, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(gy, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(gz, 2);
			UART_WriteStr(CRLF);
			UART_WriteStr("mx, my, mz: ");
			UART_WriteFloat(mx, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(my, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(mz, 2);
			UART_WriteStr(CRLF);
#if 0
			calculate_tb_angles();

			UART_WriteStr("Yaw, Pitch, Roll: ");
			UART_WriteFloat(yaw, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(pitch, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(roll, 2);
			UART_WriteStr(CRLF);

			UART_WriteStr("Grav_x, Grav_y, Grav_z: ");
			UART_WriteFloat(-a31 * 1000, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(-a32 * 1000, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(a33 * 1000, 2);
			UART_WriteStr(" mg");
			UART_WriteStr(CRLF);
			UART_WriteStr("Lin_ax, Lin_ay, Lin_az: ");
			UART_WriteFloat(lin_ax * 1000, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(lin_ay * 1000, 2);
			UART_WriteStr(", ");
			UART_WriteFloat(lin_az * 1000, 2);
			UART_WriteStr(" mg");
			UART_WriteStr(CRLF);
			
			UART_WriteStr("rate = ");
			UART_WriteFloat((float)update_cnt/elapsed_time, 2);
			UART_WriteStr(" Hz");
			UART_WriteStr(CRLF);
#endif
			last_report_time = now;
			elapsed_time = 0;
			update_cnt = 0;
		}
	}
}
