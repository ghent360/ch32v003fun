
PREFIX?=riscv64-unknown-elf

CH32V003FUN?=../../ch32v003fun
MINICHLINK?=../../minichlink
OPTFLAG?= -Os -flto
LD_LIBS?= -lgcc

CFLAGS+= \
	-g $(OPTFLAG) -fdata-sections -ffunction-sections \
	-fno-math-errno -ffast-math -ffinite-math-only \
	-static-libgcc \
	-march=rv32ec \
	-mabi=ilp32e \
	-I$(CH32V003FUN) \
	-nostdlib \
	-I. -Wall

LDFLAGS+= \
    -T $(CH32V003FUN)/ch32v003fun.ld \
	-Wl,--gc-sections \
	-L$(CH32V003FUN)/../misc \
	$(LD_LIBS)

SYSTEM_C:=$(CH32V003FUN)/ch32v003fun.c

$(TARGET).elf : $(SYSTEM_C) $(TARGET).c $(ADDITIONAL_C_FILES)
	$(PREFIX)-gcc -o $@ $^ $(CFLAGS) $(LDFLAGS)

$(TARGET).bin : $(TARGET).elf
	$(PREFIX)-size $^
	$(PREFIX)-objdump -S $^ > $(TARGET).lst
	$(PREFIX)-objdump -t $^ > $(TARGET).map
	$(PREFIX)-objcopy -O binary $< $(TARGET).bin
	$(PREFIX)-objcopy -O ihex $< $(TARGET).hex

ifeq ($(OS),Windows_NT)
closechlink :
	-taskkill /F /IM minichlink.exe /T
else
closechlink :
	-killall minichlink
endif

terminal : monitor

monitor :
	$(MINICHLINK)/minichlink -T

gdbserver : 
	-$(MINICHLINK)/minichlink -beG

cv_flash : $(TARGET).bin
	make -C $(MINICHLINK) all
	$(MINICHLINK)/minichlink -w $< flash -b

cv_clean :
	rm -rf $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).lst $(TARGET).map $(TARGET).hex || true

build : $(TARGET).bin
