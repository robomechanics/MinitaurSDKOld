
# Select the serial port (e.g. COM4, /dev/tty.usbserial-*, /dev/ttyUSB*)
PORT=COM7
# Select the robot (MINITAUR, or MINITAUR_E)
ROBOT=MINITAUR

# Don't modify below here -------------------------------------------------
ifeq ($(ROBOT),MINITAUR)
  LINKER_SCRIPT=STM32F303VC_FLASH.ld
	SDKLIB=-lmblc
else
  LINKER_SCRIPT=STM32F446ZETx_FLASH.ld
	SDKLIB=-lmbm
endif

SDK_LOCATION=../..
BFLAGS=-mcpu=cortex-m4 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O1 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra  -g -DROBOT_$(ROBOT)
CFLAGS=-DNDEBUG -DARM_MATH_CM4 -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -I$(SDK_LOCATION)/inc -I$(SDK_LOCATION)/thirdparty/nanopb 
LDFLAGS=-T $(SDK_LOCATION)/lib/$(LINKER_SCRIPT) -nostartfiles -Xlinker --gc-sections -L$(SDK_LOCATION)/lib  --specs=nano.specs -u _printf_float -Wl,-whole-archive $(SDKLIB) -Wl,-no-whole-archive
LDLIBS=-larm_cortexM4lf_math

ARCH=arm-none-eabi-

all:
	@$(ARCH)g++ $(BFLAGS) $(CFLAGS) -c -o main.o main.cpp
	@$(ARCH)g++ $(BFLAGS) $(LDFLAGS)  -o main.elf main.o $(LDLIBS)
	@$(ARCH)size --format=berkeley main.elf
	@$(ARCH)objcopy --output-target=binary main.elf main.bin
	@python $(SDK_LOCATION)/tools/stm32loader.py -p $(PORT) -b 230400 -y mblc -E 524288 -L 0 -ew main.bin

# @$(SDK_LOCATION)/tools/stm32ld.exe $(PORT) 230400 mblc main.bin 1
	
clean:
	rm -f main.elf main.bin main.o
