NAME = pullometer

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
LD = $(CXX)
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CPU = -mcpu=cortex-m0plus -mthumb
CFLAGS = $(CPU) -Os -fshort-wchar -MMD -ffast-math -fsingle-precision-constant

LDFLAGS = $(CPU) -Wl,--gc-sections --specs=nano.specs -Wl,-Map,$(NAME).map,--cref
#-u _printf_float -u _scanf_float
LIBS = -lm -lc -lgcc
#-lnosys

INCLUDES = -Icmsis -Ieigen -I.
LINKER_SCRIPT = cmsis/MKL05Z4.ld

SYS_OBJECTS = cmsis/startup_MKL05Z4.o cmsis/system_MKL05Z4.o

OBJECTS = main.o rf24.o strain.o imu.o adxl.o itg.o debugf.o support.o $(SYS_OBJECTS)

.PHONY: all size

all: $(NAME).bin size

.asm.o:
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<
.s.o:
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<
.S.o:
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<

.c.o:
	$(CC)  $(CFLAGS) -c -std=gnu99  $(INCLUDES) -o $@ $<

.cc.o:
	$(CXX) $(CFLAGS) -c -std=gnu++98 -fno-rtti -fno-exceptions $(INCLUDES) -o $@ $<

$(NAME).elf: $(OBJECTS)
	$(LD) $(LDFLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ $(LIBS)

size: $(NAME).elf
	$(SIZE) $(NAME).elf

$(NAME).bin: $(NAME).elf
	$(OBJCOPY) -O binary $< $@

upload: $(NAME).bin
	cp $(NAME).bin ~/hs/kl26/$(NAME).KL05Z.bin
clean:
	rm -f $(OBJECTS) $(NAME).elf $(NAME).bin $(NAME).map $(DEPS)

$(OBJECTS): Makefile

DEPS = $(OBJECTS:.o=.d)
-include $(DEPS)
