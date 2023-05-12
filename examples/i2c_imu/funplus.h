#pragma once

#include "ch32v003fun.h"

/* Clocks & Resets */
static inline void ABP1ClockEnable(uint32_t mask) {
    RCC->APB1PCENR |= mask;
}

static inline void ABP1ClockDisable(uint32_t mask) {
    RCC->APB1PCENR &= ~mask;
}

static inline void ABP2ClockEnable(uint32_t mask) {
    RCC->APB2PCENR |= mask;
}

static inline void ABP2ClockDisable(uint32_t mask) {
    RCC->APB2PCENR &= ~mask;
}

static inline void AHBClockEnable(uint32_t mask) {
    RCC->AHBPCENR |= mask;
}

static inline void AHBClockDisable(uint32_t mask) {
    RCC->AHBPCENR &= ~mask;
}

static inline void ABP1Reset(uint32_t mask) {
    RCC->APB1PRSTR |= mask;
    RCC->APB1PRSTR &= ~mask;
}

static inline void ABP2Reset(uint32_t mask) {
    RCC->APB2PRSTR |= mask;
    RCC->APB2PRSTR &= ~mask;
}

/* GPIO helpers */
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

static inline void GPIO_Set(GPIO_TypeDef* port, uint8_t pin) {
    GPIO_WriteMask(port, 1 << pin, 0);
}

static inline void GPIO_Clear(GPIO_TypeDef* port, uint8_t pin) {
    GPIO_WriteMask(port, 0, 1 << pin);
}

/* USART Stuff */
static inline void UART_SetupRxTx( int uartBRR )
{
	// Enable GPIOD and UART.
	ABP2ClockEnable(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1);

	// Push-Pull, 10MHz Output, GPIO D5, with Alternative Function
    // Floating input on D6.
    GPIO_Configure_Pin(GPIOD, 5, GPIO_Speed_10MHz, GPIO_CNF_OUT_PP_AF);
    GPIO_Configure_Pin(GPIOD, 6, GPIO_SPEED_IN, GPIO_CNF_IN_FLOATING);
	
	// 115200, 8n1.  Note if you don't specify a mode, UART remains off even when UE_Set.
	USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx | USART_Mode_Rx;
	USART1->CTLR2 = USART_StopBits_1;
	USART1->CTLR3 = USART_HardwareFlowControl_None;

	USART1->BRR = uartBRR;
	USART1->CTLR1 |= CTLR1_UE_Set;
}

static inline uint8_t UART_IsDataAvailable()
{
    return (USART1->STATR & USART_FLAG_RXNE);
}

static inline uint8_t UART_TxEmpty()
{
    return (USART1->STATR & USART_FLAG_TXE);
}

static inline uint8_t UART_ReadByte()
{
    while(!UART_IsDataAvailable());
    return USART1->DATAR;
}

static inline void UART_WriteByte(uint8_t data)
{
    while(!UART_TxEmpty());
    USART1->DATAR = data;
}

static inline void UART_WriteStr(const char* str)
{
    while(*str) UART_WriteByte(*str++);
}
