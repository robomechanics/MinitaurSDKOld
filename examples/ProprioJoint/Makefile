PORT=COM3

SDK_LOCATION=../..
BFLAGS=-mcpu=cortex-m4 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O1 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra  -g 
CFLAGS=-DNDEBUG -DARM_MATH_CM4 -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -I$(SDK_LOCATION)/inc -I$(SDK_LOCATION)/thirdparty/nanopb 
LDFLAGS=-T $(SDK_LOCATION)/lib/STM32F303VC_FLASH.ld -nostartfiles -Xlinker --gc-sections -L$(SDK_LOCATION)/lib  --specs=nano.specs -u _printf_float -Wl,-whole-archive -lmblc -Wl,-no-whole-archive
LDLIBS=-larm_cortexM4lf_math

ARCH=arm-none-eabi-

all:
	@$(ARCH)g++ $(BFLAGS) $(CFLAGS) -c -o main.o main.cpp
	@$(ARCH)g++ $(BFLAGS) $(LDFLAGS)  -o main.elf main.o $(LDLIBS)
	@$(ARCH)size --format=berkeley main.elf
	@$(ARCH)objcopy --output-target=binary main.elf main.bin
	# @$(SDK_LOCATION)/tools/stm32ld.exe $(PORT) 230400 mblc main.bin 1
	@python $(SDK_LOCATION)/tools/stm32loader.py -p $(PORT) -b 230400 -y mblc -E 524288 -L 0 -ew main.bin

clean:
	rm -f main.elf main.bin main.o
