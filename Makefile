BUILD = ./build
SRC   = ./src

default: compile binary flash

compile-debug: ${SRC}/main.c
	arm-none-eabi-gcc -MMD -c -mcpu=cortex-m3 -mthumb -D STM32F103xB ${SRC}/main.c -o ${BUILD}/main.o
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -D STM32F103xB ${BUILD}/main.o inc/system_stm32f1xx.o inc/startup_stm32f103xb.o -o ${BUILD}/main.elf -T inc/stm32f103c8tx.ld --specs=nosys.specs --specs=nano.specs

compile: ${SRC}/main.c
	arm-none-eabi-gcc -ggdb -MMD -c -mcpu=cortex-m3 -mthumb -D STM32F103xB ${SRC}/main.c -o ${BUILD}/main.o
	arm-none-eabi-gcc -ggdb -mcpu=cortex-m3 -mthumb -D STM32F103xB ${BUILD}/main.o inc/system_stm32f1xx.o inc/startup_stm32f103xb.o -o ${BUILD}/main.elf -T inc/stm32f103c8tx.ld --specs=nosys.specs --specs=nano.specs

binary: ${BUILD}/main.elf
	arm-none-eabi-objcopy ${BUILD}/main.elf ${BUILD}/main.bin -O binary

clean:
	rm -rf ${BUILD}/*

flash: ${BUILD}/main.bin
	st-flash write ${BUILD}/main.bin 0x08000000