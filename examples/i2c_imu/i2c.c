#define SYSTEM_CORE_CLOCK 48000000
#define APB_CLOCK SYSTEM_CORE_CLOCK

#include "ch32v003fun.h"
#include "funplus.h"
#include "i2c.h"

#ifdef I2C_USE_IRQ
// some stuff that IRQ mode needs
static uint8_t i2c_send_buffer[64];
uint8_t *i2c_send_ptr;
uint8_t i2c_send_sz;
uint8_t i2c_irq_state;
#endif

static inline uint32_t i2c_get_evt()
{
	/* read order matters here! STAR1 before STAR2!! */
	return I2C1->STAR1 | (I2C1->STAR2<<16);
}

/*
 * check for 32-bit event codes
 */
static inline uint8_t i2c_chk_evt(uint32_t event_mask)
{
	/* read order matters here! STAR1 before STAR2!! */
	uint32_t status = i2c_get_evt();
	return (status & event_mask) == event_mask;
}

/*
 * Wait for the I2C bus to become free
 */
static inline uint8_t wait_i2c_free() {
	uint32_t timeout = TIMEOUT_MAX;
	
	do
	{
		if ((I2C1->STAR2 & I2C_STAR2_BUSY) == 0) return 0;
		timeout--;
	} while (timeout);
	return 1;
}

/*
 * Check Address write transmission for errors or completion.
 */
static inline uint8_t i2c_chk_addr_write()
{
	uint32_t timeout = TIMEOUT_MAX;
	
	do
	{
		uint32_t status = i2c_get_evt();

		if ((status & (I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF)) != 0) return 2;
		if ((status & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) ==
		    I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) return 0;
		timeout--;
	} while (timeout);
	return 1;
}

/*
 * Check Address read transmission for errors or completion.
 */
static inline uint8_t i2c_chk_addr_read()
{
	uint32_t timeout = TIMEOUT_MAX;
	
	do
	{
		uint32_t status = i2c_get_evt();

		if ((status & (I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF)) != 0) return 2;
		if ((status & I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) ==
		    I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) return 0;
		timeout--;
	} while (timeout);
	return 1;
}

/*
 * Wait for I2C event
 */
static inline uint8_t wait_i2c_evt(uint32_t event_mask) {
	uint32_t timeout = TIMEOUT_MAX;
	
	do
	{
		if (i2c_chk_evt(event_mask)) return 0;
		timeout--;
	} while (timeout);
	return 1;
}

/*
 * Wait for I2C TX register to become empty
 */
static inline uint8_t wait_i2c_tx_empty() {
	uint32_t timeout = TIMEOUT_MAX;
	
	do
	{
		uint16_t star1 = I2C1->STAR1;
		if ((star1 & (I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF)) != 0) return 2;
		if ((star1 & I2C_STAR1_TXE) == I2C_STAR1_TXE) return 0;
		timeout--;
	} while (timeout);
	return 1;
}

/*
 * Wait for I2C RX register to become not empty
 */
static inline uint8_t wait_i2c_rx_not_empty() {
	uint32_t timeout = TIMEOUT_MAX;
	
	do
	{
		uint16_t star1 = I2C1->STAR1;
		if ((star1 & (I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF)) != 0) return 2;
		if ((star1 & I2C_STAR1_RXNE) == I2C_STAR1_RXNE) return 0;
		timeout--;
	} while (timeout);
	return 1;
}

#ifdef I2C_USE_IRQ
/*
 * packet send for IRQ-driven operation
 */
uint8_t i2c_write(uint8_t addr, const uint8_t *data, uint8_t sz)
{
	int32_t timeout;
	
#ifdef IRQ_DIAG
	GPIOC->BSHR = (1<<(3));
#endif
	
	// error out if buffer under/overflow
	if((sz > sizeof(i2c_send_buffer)) || !sz)
		return 2;
	
	// wait for previous packet to finish
	while(i2c_irq_state);
	
#ifdef IRQ_DIAG
	GPIOC->BSHR = (1<<(16+3));
	GPIOC->BSHR = (1<<(4));
#endif
	
	// init buffer for sending
	i2c_send_sz = sz;
	i2c_send_ptr = i2c_send_buffer;
	memcpy((uint8_t *)i2c_send_buffer, data, sz);
	
	// wait for not busy
	timeout = TIMEOUT_MAX;
	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
	if(timeout==-1)
		return ssd1306_i2c_error(0);

	// Set START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;

	// wait for master mode select
	timeout = TIMEOUT_MAX;
	while((!ssd1306_i2c_chk_evt(SSD1306_I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));
	if(timeout==-1)
		return ssd1306_i2c_error(1);
	
	// send 7-bit address + write flag
	I2C1->DATAR = addr<<1;

	// wait for transmit condition
	timeout = TIMEOUT_MAX;
	while((!ssd1306_i2c_chk_evt(SSD1306_I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));
	if(timeout==-1)
		return ssd1306_i2c_error(2);

	// Enable TXE interrupt
	I2C1->CTLR2 |= I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN;
	i2c_irq_state = 1;

#ifdef IRQ_DIAG
	GPIOC->BSHR = (1<<(16+4));
#endif
	
	// exit
	return 0;
}

/*
 * IRQ handler for I2C events
 */
void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void)
{
	uint16_t STAR1, STAR2 __attribute__((unused));
	
#ifdef IRQ_DIAG
	GPIOC->BSHR = (1<<(4));
#endif

	// read status, clear any events
	STAR1 = I2C1->STAR1;
	STAR2 = I2C1->STAR2;
	
	/* check for TXE */
	if(STAR1 & I2C_STAR1_TXE)
	{
		/* check for remaining data */
		if(i2c_send_sz--)
			I2C1->DATAR = *i2c_send_ptr++;

		/* was that the last byte? */
		if(!i2c_send_sz)
		{
			// disable TXE interrupt
			I2C1->CTLR2 &= ~(I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN);
			
			// reset IRQ state
			i2c_irq_state = 0;
			
			// wait for tx complete
			while(!ssd1306_i2c_chk_evt(SSD1306_I2C_EVENT_MASTER_BYTE_TRANSMITTED));

			// set STOP condition
			I2C1->CTLR1 |= I2C_CTLR1_STOP;
		}
	}

#ifdef IRQ_DIAG
	GPIOC->BSHR = (1<<(16+4));
#endif
}
#else
/*
 * low-level packet send for blocking polled operation via i2c
 */
static inline uint8_t i2c_write_internal(
	uint8_t addr, const uint8_t *data, uint8_t sz, uint8_t should_stop)
{
	// wait for not busy
	if (wait_i2c_free()) return 1;
    // Set START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;
	// wait for master mode select
	if (wait_i2c_evt(I2C_EVENT_MASTER_MODE_SELECT)) {
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
		return 2;
	}
	// send 7-bit address + write flag
	I2C1->DATAR = addr << 1;
	// wait for transmit condition
	if (i2c_chk_addr_write()) {
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
		return 3;
	}

	// send data one byte at a time
	while(sz--)
	{
		// wait for TX Empty
		if (wait_i2c_tx_empty()) {
			I2C1->CTLR1 |= I2C_CTLR1_STOP;
			return 4;
		}
		// send next byte
		I2C1->DATAR = *data++;
	}

	// wait for tx complete on the last byte
	if (wait_i2c_evt(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
		return 5;
	}

	if (should_stop)
	{
		// set STOP condition
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
	}

	// we're happy
	return 0;
}

uint8_t i2c_write(
	uint8_t addr, const uint8_t *data, uint8_t sz)
{
	return i2c_write_internal(addr, data, sz, 1);
}

uint8_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
	uint8_t data[2] = {reg, value};
	return i2c_write(addr, data, sizeof(data));
}

/*
 * low-level packet read for blocking polled operation via i2c
 */
uint8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t sz)
{
	i2c_write_internal(addr, &reg, 1, 0);

    // Set START condition and enable ACK of received bytes
	I2C1->CTLR1 |= I2C_CTLR1_START | I2C_CTLR1_ACK;

	// wait for master mode select
	if (wait_i2c_evt(I2C_EVENT_MASTER_MODE_SELECT)) {
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
		return 2;
	}

	// send 7-bit address + read flag
	I2C1->DATAR = (addr << 1) | 1;
	// wait for receive condition
	if (i2c_chk_addr_read()) {
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
		return 3;
	}

	// read data one byte at a time
	while(sz--)
	{
		// Send NACK for the last byte
		if (!sz) {
			I2C1->CTLR1 &= ~(I2C_CTLR1_ACK);
		}
		// wait for RX not empty
		if (wait_i2c_rx_not_empty()) {
			I2C1->CTLR1 |= I2C_CTLR1_STOP;
			return 4;
		}
		// read next byte
		*data++ = I2C1->DATAR;
	}

	// set STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;

	// we're happy
	return 0;
}

uint8_t i2c_read_reg(uint8_t addr, uint8_t reg)
{
	uint8_t data = 0;
	i2c_read(addr, reg, &data, 1);
	return data;
}
#endif

/*
 * init I2C and GPIO
 */
void i2c_init_master(void)
{
	uint16_t tempreg;

	// Enable GPIOC and I2C
	ABP2ClockEnable(RCC_APB2Periph_GPIOC);
	ABP1ClockEnable(RCC_APB1Periph_I2C1);
	
	// PC1 is SDA, 10MHz Output, alt func, open-drain
	// PC2 is SCL, 10MHz Output, alt func, open-drain
	GPIO_Configure_Pin(GPIOC, 1, GPIO_Speed_10MHz, GPIO_CNF_OUT_OD_AF);
	GPIO_Configure_Pin(GPIOC, 2, GPIO_Speed_10MHz, GPIO_CNF_OUT_OD_AF);

	// Reset I2C1 to init all regs
	ABP1Reset(RCC_APB1Periph_I2C1);
	
	// set freq
	tempreg = I2C1->CTLR2;
	tempreg &= ~I2C_CTLR2_FREQ;
	tempreg |= (APB_CLOCK/I2C_CLKRATE) & I2C_CTLR2_FREQ;
	I2C1->CTLR2 = tempreg;
	
	// Set clock config
	tempreg = 0;
#if (I2C_BUSRATE <= 100000)
	// standard mode good to 100kHz
	tempreg = (APB_CLOCK/(2*I2C_BUSRATE)) & I2C_CKCFGR_CCR;
#if 0
	if (tempreg < 4)
	{
		tempreg = 4;
	}
#endif
#else
	// fast mode over 100kHz
#ifndef I2C_DUTY
	// 33% duty cycle
	tempreg = (APB_CLOCK/(3*I2C_BUSRATE)) & I2C_CKCFGR_CCR;
#else
	// 36% duty cycle
	tempreg = (APB_CLOCK/(25*I2C_BUSRATE)) & I2C_CKCFGR_CCR;
	tempreg |= I2C_CKCFGR_DUTY;
#endif
#if 0
    if((tempreg & I2C_CKCFGR_CCR) == 0)
    {
        tempreg |= 1;
    }
#endif
	tempreg |= I2C_CKCFGR_FS;
#endif
	I2C1->CKCFGR = tempreg;

#ifdef I2C_USE_IRQ
	// enable IRQ driven operation
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	
	// initialize the state
	i2c_irq_state = 0;
#endif
	
	// Enable I2C
	I2C1->CTLR1 |= I2C_CTLR1_PE;
}
