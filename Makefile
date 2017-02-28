CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
LIBOPENCM3 ?= ./libopencm3
LIBNRF24L01 ?= ./libemb/libnrf24l01/src

APP_ADDRESS = 0x08000000
APP_OFFSET = $(shell echo $$(($(APP_ADDRESS) - 0x08000000)))

CFLAGS = -Os -std=gnu99 -Wall -pedantic -Istm32/include \
	-mcpu=cortex-m3 -mthumb -DSTM32F1 \
	-I$(LIBOPENCM3)/include -I$(LIBNRF24L01)/include -DAPP_ADDRESS=$(APP_ADDRESS) -ggdb3

LDFLAGS = -lopencm3_stm32f1 \
	-Wl,-Tstm32f103.ld -nostartfiles -lc -lnosys \
	-mthumb -mcpu=cortex-m3 -L$(LIBOPENCM3)/lib/ -L$(LIBNRF24L01)/ -Wl,-gc-sections

stm32-nrf24l01-hid.elf: stm32-nrf24l01-hid.o | $(LIBOPENCM3)/lib/libopencm3_stm32f1.a
	$(CC) $^ -o $@ $(LDFLAGS) -Wl,-Ttext=$(APP_ADDRESS)

$(LIBOPENCM3)/lib/libopencm3_stm32f1.a:
	$(MAKE) -C $(LIBOPENCM3) TARGETS=stm32/f1

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

clean:
	-rm *.elf *.o
