all : flash

TARGET:=run_from_ram

CFLAGS+=-DTINYVECTOR

include ../../ch32v003fun/ch32v003fun.mk

flash : cv_flash
clean : cv_clean

