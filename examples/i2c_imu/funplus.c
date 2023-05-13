#define SYSTEM_CORE_CLOCK 48000000
#define APB_CLOCK SYSTEM_CORE_CLOCK
#include "funplus.h"

const char CRLF[] = "\r\n";

void UART_WriteUInt(uint32_t value) {
    char buffer[10];
    uint8_t idx = 0;
    do {
        buffer[idx++] = (value % 10) + '0';
        value /= 10;
    } while (value > 0);
    while (idx > 0) {
      UART_WriteByte(buffer[--idx]);
    }
}

static inline char byteToHex(uint8_t v) {
    if (v > 9) return 'A' + v - 10;
    return '0' + v;
}

void UART_WriteHex(uint32_t value) {
    int32_t shift = 4;
    if (value > 0xff) shift = 12;
    if (value > 0xffff) shift = 28;
    do {
        UART_WriteByte(byteToHex((value >> shift) & 0x0F));
        shift -= 4;
    } while (shift >= 0);
}

void UART_WriteInt(int32_t value) {
    if (value < 0) {
        UART_WriteByte('-');
        value = -value;
    }
    UART_WriteUInt((uint32_t)value);
}

static const uint32_t pow10[] = {
    1,
    10,
    100,
    1000,
    10000,
    100000,
    1000000
};

void UART_WriteFloat(float value, uint8_t dec_points) {
    if (value < 0) {
        UART_WriteByte('-');
        value = -value;
    }
    if (dec_points > 6) dec_points = 6;
    if (dec_points == 0) {
        uint32_t v = (uint32_t)(value + 0.5f);
        UART_WriteUInt(v);
    } else {
        uint32_t v = (uint32_t)value;
        UART_WriteUInt(v);
        UART_WriteByte('.');
        v = (uint32_t)((value - (float)v) * (float)pow10[dec_points] + 0.5f);
        UART_WriteUInt(v);
    }
}
