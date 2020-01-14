BUILD = ./build
SRC   = ./src

OPTIMIZATION = -O0
SOURCES_ASM = $(wildcard inc/*.o)
SOURCES_C = $(wildcard src/*.c src/*/*.c)
OBJECTS = ${patsubst src%, build%, $(SOURCES_C:.c=.o)} ${patsubst inc%, build%, $(SOURCES_ASM)}
AS = arm-none-eabi-as -mcpu=cortex-m3 -mthumb
LD = arm-none-eabi-gcc $(OPTIMIZATION) -mcpu=cortex-m3 -mthumb -D STM32F103xB --specs=nosys.specs --specs=nano.specs
GCC = arm-none-eabi-gcc $(OPTIMIZATION) -ggdb -MMD -c -mcpu=cortex-m3 -mthumb -D STM32F103xB
SIZE = arm-none-eabi-size

default: copy compile binary flash size
production: copy compile-production binary flash size

# colors
green_col = @echo -en "\u001b[1;32m"
whtie_col = @echo -en "\033[m\u001b[37m"
red_col = @echo -en "\u001b[1;31m"

compile-production: $(OBJECTS)
	$(green_col)
	@echo "COMPILING FOR PRODUCTION:"
	$(whtie_col)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -D STM32F103xB ${OBJECTS} -o ${BUILD}/main.elf -T inc/stm32f103c8tx.ld --specs=nosys.specs --specs=nano.specs

copy:
	cp -rf src/* inc/*.o build/

compile: $(OBJECTS)
	$(green_col)
	@echo "COMPILING:"
	$(whtie_col)
	arm-none-eabi-gcc -ggdb -mcpu=cortex-m3 -mthumb -D STM32F103xB ${OBJECTS} -o ${BUILD}/main.elf -T inc/stm32f103c8tx.ld --specs=nosys.specs --specs=nano.specs

binary: ${BUILD}/main.elf
	arm-none-eabi-objcopy ${BUILD}/main.elf ${BUILD}/main.bin -O binary

%.o: %.s
	$(AS) $< -o $@

%.o: %.asm
	$(AS) $< -o $@

%.o: %.c
	$(GCC) $< -o $@

clean:
	rm -rf ${BUILD}/*
	$(red_col)
	@echo "CLEANED!"
	$(white_col)

flash: ${BUILD}/main.bin
	$(green_col)
	@echo "FLASHING:"
	$(whtie_col)
	st-flash write ${BUILD}/main.bin 0x08000000

size: ${BUILD}/main.elf
	$(green_col)
	@echo "SIZES:"
	@$(SIZE) $(BUILD)/main.elf
