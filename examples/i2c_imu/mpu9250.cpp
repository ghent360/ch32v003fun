#include <stdint.h>
#include <math.h>
#include "funplus.h"
#include "mpu9250.h"
#include "quaternion_filters.h"
#include "i2c.h"

constexpr uint16_t gyro_sensitivity  = 131;    // = 131 LSB/degrees/sec
constexpr uint16_t accel_sensitivity = 16384;  // = 16384 LSB/g

MPU9250Gscale g_scale = GFS_250DPS;
MPU9250Ascale a_scale = AFS_2G;
MPU9250Mscale m_scale = MFS_16BITS;
uint8_t m_mode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

static float a_res, g_res, m_res;// scale resolutions per LSB for the sensors
static float mag_calibration[3];  // Factory mag calibration and mag bias

static float gyro_bias_values[3];
static float accel_bias_values[3];
static float mag_bias_values[3];

static inline float getMres() {
  return (m_scale == MFS_14BITS) ?
    (10.0f * 4912.0f / 8190.f) :
    (10.0f * 4912.0f / 32760.0f);
}

static inline float getGres() {
  switch (g_scale) {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_250DPS:
        return (250.0f / 32768.0f);
    case GFS_500DPS:
        return (500.0f / 32768.0f);
    case GFS_1000DPS:
        return (1000.0f / 32768.0f);
    default:;
  }
  return (2000.0f / 32768.0f);
}

static inline float getAres() {
    switch (a_scale) {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    case AFS_2G:
        return (2.0f / 32768.0f);
    case AFS_4G:
        return (4.0f / 32768.0f);
    case AFS_8G:
        return (8.0f / 32768.0f);
    default:;
    }
    return (16.0f / 32768.0f);
}

void MPU9250_initRes() {
   a_res = getAres();
   g_res = getGres();
   m_res = getMres();
}

static void MPU9250_readAccelData() {
    uint8_t raw_data[6];  // x/y/z accel register data stored here
    // Read the six raw data registers into data array
    i2c_read(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, raw_data, 6);
    // Turn the MSB and LSB into a signed 16-bit value
    // get actual g value, this depends on scale being set
    ax = (float)(((int16_t)raw_data[0] << 8) | raw_data[1]) * a_res;
     //- accel_bias_values[0];
    ay = (float)(((int16_t)raw_data[2] << 8) | raw_data[3]) * a_res;
     //- accel_bias_values[1];   
    az = (float)(((int16_t)raw_data[4] << 8) | raw_data[5]) * a_res;
     //- accel_bias_values[2];  
}

static void MPU9250_readGyroData() {
    uint8_t raw_data[6];  // x/y/z gyro register data stored here
    // Read the six raw data registers sequentially into data array
    i2c_read(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, raw_data, 6);

    // Turn the MSB and LSB into a signed 16-bit value
    // get actual gyro value, this depends on scale being set
    gx = (float)(((int16_t)raw_data[0] << 8) | raw_data[1]) * g_res;  
    gy = (float)(((int16_t)raw_data[2] << 8) | raw_data[3]) * g_res;  
    gz = (float)(((int16_t)raw_data[4] << 8) | raw_data[5]) * g_res;   
}

static void MPU9250_readMagData(int16_t *destination) {
    // x/y/z gyro register data, ST2 register stored here
    // must read ST2 at end of data acquisition
    uint8_t raw_data[7];
    // wait for magnetometer data ready bit to be set
    i2c_read_reg(AK8963_ADDRESS, AK8963_ST1, raw_data);
    if (raw_data[0] & 0x01) {
        // Read the six raw data and ST2 registers sequentially into data array
        i2c_read(AK8963_ADDRESS, AK8963_XOUT_L, raw_data, 7);
        // End data read by reading ST2 register
        if(!(raw_data[6] & 0x08)) {
            // Check if magnetic sensor overflow set, if not then report data
            // Turn the MSB and LSB into a signed 16-bit value
            destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];
            destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];
            destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];
        }
    }
}

static void MPU9250_readMagData() {
    // x/y/z gyro register data, ST2 register stored here
    // must read ST2 at end of data acquisition
    uint8_t raw_data[7];
    // wait for magnetometer data ready bit to be set
    i2c_read_reg(AK8963_ADDRESS, AK8963_ST1, raw_data);
    if (raw_data[0] & 0x01) {
        // Read the six raw data and ST2 registers sequentially into data array
        i2c_read(AK8963_ADDRESS, AK8963_XOUT_L, raw_data, 7);
        // End data read by reading ST2 register
        if(!(raw_data[6] & 0x08)) {
            // Check if magnetic sensor overflow set, if not then report data
            // Turn the MSB and LSB into a signed 16-bit value
            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            // get actual magnetometer value, this depends on scale being set
            mx = (float)(((int16_t)raw_data[1] << 8) | raw_data[0]) * mag_calibration[0] - mag_bias_values[0];
            my = (float)(((int16_t)raw_data[3] << 8) | raw_data[2]) * mag_calibration[1] - mag_bias_values[1];  
            mz = (float)(((int16_t)raw_data[5] << 8) | raw_data[4]) * mag_calibration[2] - mag_bias_values[2];
        }
    }
}

int16_t MPU9250_readTempData() {
    uint8_t raw_data[2];
    // Read the two raw data registers sequentially into data array 
    i2c_read(MPU9250_ADDRESS, MPU9250_TEMP_OUT_H, raw_data, 2);
    // Turn the MSB and LSB into a 16-bit value
    return ((int16_t)raw_data[0] << 8) | raw_data[1];
}

void AK8963_init() {
    // First extract the factory calibration for each magnetometer axis
    uint8_t raw_data[3];  // x/y/z gyro calibration data stored here
    i2c_write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
    Delay_Ms(10);
    i2c_write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    Delay_Ms(10);
    // Read the x-, y-, and z-axis calibration values
    i2c_read(AK8963_ADDRESS, AK8963_ASAX, raw_data, 3);
    // Return x-axis sensitivity adjustment values, etc.
    mag_calibration[0] = ((float)(raw_data[0] - 128) / 256.0f + 1.0f) * m_res;
    mag_calibration[1] = ((float)(raw_data[1] - 128) / 256.0f + 1.0f) * m_res;  
    mag_calibration[2] = ((float)(raw_data[2] - 128) / 256.0f + 1.0f) * m_res; 
    i2c_write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
    Delay_Ms(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]),
    // 0010 for 8 Hz and 0110 for 100 Hz sample rates
    // Set magnetometer data resolution and sample ODR
    i2c_write_reg(AK8963_ADDRESS, AK8963_CNTL, (m_scale << 4) | m_mode);
    Delay_Ms(10);
}

void MPU9250_init()
{  
    // wake up device
    // Clear sleep mode bit (6), enable all sensors
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);
    Delay_Ms(100); // Wait for all registers to reset 

    // get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);
    Delay_Ms(200); 

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum Delay_Ms time for this setting is 5.9 ms, which means sensor fusion
    // update rates cannot be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz,
    // or 1 kHz
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate 
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x04);
    // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3,
    // so 2-bit values are left-shifted into positions 4:3
    uint8_t c;
    i2c_read_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, &c);

    // Clear self-test bits [7:5]
    // i2c_write_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c & ~0xE0);

    // Clear Fchoice bits [1:0] 
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c & ~0x03);
    // Clear GFS bits [4:3]
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c & ~0x18);
    // Set full scale range for the gyro
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c | (g_scale << 3));
    // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    // i2c_write_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c | 0x00);

    // Set accelerometer full-scale range configuration
    i2c_read_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, &c);
    // Clear self-test bits [7:5]
    //  i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c & ~0xE0);
    // Clear AFS bits [4:3]
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c & ~0x18);
    // Set full scale range for the accelerometer 
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c | (a_scale << 3));

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    i2c_read_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, &c);
    // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, c & ~0x0F);
    // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, c | 0x03);

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because
    // of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of INT_STATUS, and enable
    // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
    // controlled by the MCU as master
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x22);
    // Enable data ready (bit 0) interrupt
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x01);
    Delay_Ms(100);
}

// Function which accumulates gyro and accelerometer data after 
// device initialization. It calculates the average of the at-rest
// readings and then loads the resulting offsets into accelerometer
// and gyro bias registers.
void MPU9250_calibrate_accelgyro()
{  
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0};
    int32_t accel_bias[3] = {0, 0, 0};

    // reset device
    // Write a one to bit 7 reset bit; toggle reset device
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80);
    Delay_Ms(100);

    // get stable time source; Auto select clock source to be PLL
    // gyroscope reference if ready else use the internal oscillator,
    // bits 2:0 = 001
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);  
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, 0x00);
    Delay_Ms(200);                                    

    // Configure device for bias calculation
    // Disable all interrupts
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x00);
    // Disable FIFO
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);
    // Turn on internal clock source
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);
    // Disable I2C master
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_I2C_MST_CTRL, 0x00);
    // Disable FIFO and I2C master modes
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x00);
    // Reset FIFO and DMP
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x0C);
    Delay_Ms(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    // Set low-pass filter to 188 Hz
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);
    // Set sample rate to 1 kHz
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);
    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00);
    // Set accelerometer full-scale to 2 g, maximum sensitivity
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    // Enable FIFO
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x40);
    // Enable gyro and accelerometer sensors for FIFO
    // (max size 512 bytes in MPU-9150)
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x78);
    Delay_Ms(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    // Disable gyro and accelerometer sensors for FIFO
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);
    // read FIFO sample count
    i2c_read(MPU9250_ADDRESS, MPU9250_FIFO_COUNTH, data, 2);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    // How many sets of full gyro and accelerometer data for averaging
    packet_count = fifo_count / 12;

    for (ii = 0; ii < packet_count; ii++) {
        int16_t temp;
        // read data for averaging
        i2c_read(MPU9250_ADDRESS, MPU9250_FIFO_R_W, data, 12);
        // Form signed 16-bit integer for each sample in FIFO
        temp = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
        accel_bias[0] += (int32_t) temp;
        temp = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
        accel_bias[1] += (int32_t) temp;
        temp = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
        accel_bias[2] += (int32_t) temp;
        temp  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
        gyro_bias[0]  += (int32_t) temp;
        temp  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
        gyro_bias[1]  += (int32_t) temp;
        temp  = (int16_t) (((int16_t)data[10] << 8) | data[11]);
        gyro_bias[2]  += (int32_t) temp;
    }
    // Normalize sums to get average count biases
    accel_bias[0] /= (int32_t)packet_count;
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0]  /= (int32_t)packet_count;
    gyro_bias[1]  /= (int32_t)packet_count;
    gyro_bias[2]  /= (int32_t)packet_count;

    if (accel_bias[2] > 0L) {
        // Remove gravity from the z-axis accelerometer bias calculation
        accel_bias[2] -= (int32_t) accel_sensitivity;
    } else {
        accel_bias[2] += (int32_t) accel_sensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = MPU9250_XG_OFFSET_H;
    data[1] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[2] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[3] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[4] = (-gyro_bias[1]/4)       & 0xFF;
    data[5] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[6] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    i2c_write(MPU9250_ADDRESS, data, 7);
    /*
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[0]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[1]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[2]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[3]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[4]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[5]);
    */

    // Output scaled gyro biases for display in the main program
    gyro_bias_values[0] = (float) gyro_bias[0]/(float) gyro_sensitivity;  
    gyro_bias_values[1] = (float) gyro_bias[1]/(float) gyro_sensitivity;
    gyro_bias_values[2] = (float) gyro_bias[2]/(float) gyro_sensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    // A place to hold the factory accelerometer trim biases
    int32_t accel_bias_reg[3];
    // Read factory accelerometer trim values
    i2c_read(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, data, 6);
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[2] << 8) | data[3]);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[4] << 8) | data[5]);

    // Define mask for temperature compensation bit 0 of lower byte of
    // accelerometer bias registers.
    uint32_t mask = 1uL;
    // Define array to hold mask bit for each accelerometer bias axis
    uint8_t mask_bit[3];

    for (ii = 0; ii < 3; ii++) {
        if ((accel_bias_reg[ii] & mask)) {
            // If temperature compensation bit is set, record that fact in mask_bit
            mask_bit[ii] = 0x01;
        }
        accel_bias_reg[ii] >>= 1;
    }

    // Construct total accelerometer bias, including calculated average
    // accelerometer bias from above
    // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
    // (16 g full scale)
    accel_bias_reg[0] -= (accel_bias[0]/8);
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    // Output scaled accelerometer biases for display in the main program
    accel_bias[0] = (float)accel_bias[0]/(float)accel_sensitivity; 
    accel_bias[1] = (float)accel_bias[1]/(float)accel_sensitivity;
    accel_bias[2] = (float)accel_bias[2]/(float)accel_sensitivity;

    for (ii = 0; ii < 3; ii++) {
        accel_bias_reg[ii] <<= 1;
    }

    data[0] = MPU9250_XA_OFFSET_H;
    data[1] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[2] = (accel_bias_reg[0])      & 0xFE;
    // preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[2] |= mask_bit[0];
    data[3] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[4] = (accel_bias_reg[1])      & 0xFE;
    // preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[4] |= mask_bit[1];
    data[5] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[6] = (accel_bias_reg[2])      & 0xFE;
    // preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[6] |= mask_bit[2];

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    i2c_write(MPU9250_ADDRESS, data, 7);
    /*
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, data[0]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_XA_OFFSET_L, data[1]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, data[2]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_YA_OFFSET_L, data[3]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, data[4]);
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_L, data[5]);
    */
   accel_bias_values[0] = (float)accel_bias[0]/(float)accel_sensitivity; 
   accel_bias_values[1] = (float)accel_bias[1]/(float)accel_sensitivity;
   accel_bias_values[2] = (float)accel_bias[2]/(float)accel_sensitivity;
}

void MPU9250_calibrate_mag() {
    uint16_t ii;
    uint16_t sample_count;
    int16_t mag_bias[3];
    int16_t mag_max[3] = {-32767, -32767, -32767};
    int16_t mag_min[3] = {32767, 32767, 32767};

    //Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    //Delay_Ms(4000);

    sample_count = 64;
    for (ii = 0; ii < sample_count; ii++) {
        MPU9250_readMagData(mag_bias);  // Read the mag data   
        for (int jj = 0; jj < 3; jj++) {
            if (mag_bias[jj] > mag_max[jj])
                mag_max[jj] = mag_bias[jj];
            if (mag_bias[jj] < mag_min[jj])
                mag_min[jj] = mag_bias[jj];
        }
        Delay_Ms(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    }

    //    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    //    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    //    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    // save mag biases in G for main program
    mag_bias_values[0] = (float) mag_bias[0] * m_res * mag_calibration[0];
    mag_bias_values[1] = (float) mag_bias[1] * m_res * mag_calibration[1];   
    mag_bias_values[2] = (float) mag_bias[2] * m_res * mag_calibration[2];          

    //Serial.println("Mag Calibration done!");
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or
// less deviation is a pass
void MPU9250_self_test(float * destination) 
{
    uint8_t raw_data[6];
    int32_t gAvg[3] = {0};
    int32_t aAvg[3] = {0};
    int32_t aSTAvg[3] = {0};
    int32_t gSTAvg[3] = {0};
    float factoryTrim[6];
    constexpr uint8_t FS = 0;

    // Set gyro sample rate to 1 kHz
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);
    // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);
    // Set full scale range for the gyro to 250 dps
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1 << FS);
    // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x02);
    // Set full scale range for the accelerometer to 2 g
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1 << FS);

    // get average current values of gyro and acclerometer
    for (uint8_t ii = 0; ii < 200; ii++) {
        // Read the six raw data registers into data array
        i2c_read(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, raw_data, 6);
        // Turn the MSB and LSB into a signed 16-bit value
        aAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
        aAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        aAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

        // Read the six raw data registers sequentially into data array
        i2c_read(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, raw_data, 6);
        // Turn the MSB and LSB into a signed 16-bit value
        gAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
        gAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        gAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
    }

    // Get average of 200 values and store as average current readings
    for (uint8_t ii = 0; ii < 3; ii++) {
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    // Enable self test on all three axes and set accelerometer range to +/- 2 g
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0xE0);
    // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0xE0);
    Delay_Ms(25);  // Delay a while to let the device stabilize

    // get average self-test values of gyro and acclerometer
    for (uint8_t ii = 0; ii < 200; ii++) {
        // Read the six raw data registers into data array
        i2c_read(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, raw_data, 6);
        // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
        aSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        aSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

        // Read the six raw data registers sequentially into data array
        i2c_read(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, raw_data, 6);
        // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
        gSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        gSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
    }

    // Get average of 200 values and store as average self-test readings
    for (uint8_t ii = 0; ii < 3; ii++) {
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }   

    // Configure the gyro and accelerometer for normal operation
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);  
    i2c_write_reg(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0x00);  
    Delay_Ms(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    // Accel self-test results
    i2c_read(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL, raw_data, 3);
    // Gyro self-test results
    i2c_read(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO, &raw_data[3], 3);

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620/(1<<FS))*(powf( 1.01f , ((float)raw_data[0] - 1.0f) )); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620/(1<<FS))*(powf( 1.01f , ((float)raw_data[1] - 1.0f) )); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620/(1<<FS))*(powf( 1.01f , ((float)raw_data[2] - 1.0f) )); // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620/(1<<FS))*(powf( 1.01f , ((float)raw_data[3] - 1.0f) )); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620/(1<<FS))*(powf( 1.01f , ((float)raw_data[4] - 1.0f) )); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620/(1<<FS))*(powf( 1.01f , ((float)raw_data[5] - 1.0f) )); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (uint8_t i = 0; i < 3; i++) {
        destination[i]   = 100.0f*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.0f;   // Report percent differences
        destination[i + 3] = 100.0f*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.0f; // Report percent differences
    }
}

uint8_t read_imu_data() {
    uint8_t data;
    i2c_read_reg(MPU9250_ADDRESS, MPU9250_INT_STATUS, &data);
    if (data & 0x01) {  // check if data ready interrupt
        MPU9250_readAccelData();
        MPU9250_readGyroData();
        MPU9250_readMagData();
        return 0;
    }
    return 1;
}