all : flash

TARGET:=i2c_imu
ADDITIONAL_C_FILES:=i2c.c quaternion_filters.cpp
OPTFLAG:=-Os -flto
LD_LIBS:=-lgcc -lm -lgcc
PREFIX:=riscv-none-elf

#CFLAGS+=-DSTDOUT_UART

include ../../ch32v003fun/ch32v003fun.mk

flash : cv_flash
clean : cv_clean

