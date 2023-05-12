#pragma once

// I2C Bus clock rate - must be lower the Logic clock rate
#define I2C_BUSRATE 400000

// I2C module clock rate - must be higher than Bus clock rate
#define I2C_CLKRATE 1000000

// uncomment this for high-speed 36% duty cycle, otherwise 33%
//#define I2C_DUTY

// I2C Timeout count
#define TIMEOUT_MAX 10000

// uncomment this to enable IRQ-driven operation
//#define I2C_USE_IRQ

void i2c_init_master(void);
uint8_t i2c_write(uint8_t addr, const uint8_t *data, uint8_t sz);
uint8_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t sz);
uint8_t i2c_read_reg(uint8_t addr, uint8_t reg);