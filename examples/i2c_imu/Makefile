all : flash

TARGET:=i2c_imu
ADDITIONAL_C_FILES:=i2c.c quaternion_filters.cpp mpu9250.cpp funplus.c systick.c
#OPTFLAG:=-Og
LD_LIBS:=-lgcc -lm -lgcc
PREFIX:=riscv-none-elf
CFLAGS+= \
  -DSYSTEM_CORE_CLOCK=48000000 \
  -DAPB_CLOCK=SYSTEM_CORE_CLOCK \
  -DSYSTICK_USE_HCLK

include ../../ch32v003fun/ch32v003fun.mk

flash : cv_flash
clean : cv_clean

