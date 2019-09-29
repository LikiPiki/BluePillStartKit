# STM32 Blue pill empty project

## Installing
Installing dependencies. Also you need install VSCode.
```
brew install arm-none-eabi-gcc stlink
```
Installing Vscode plugins
- Cortex debug
- ARM
- C/C++

## Building project

```
make - make and flash stm32 blue pill
make flash - flash only
make build - build only
```

### Debug with VSCode (Cortex debug plugin)
All debugger congigurations you can find in `.vscode/launch.json`
Default configuration:
**For debugging, you need `set DEBUG=yes` in Makefile, to compile with -ggdb option** - TODO
**You also need .SVD file to you microcontroller , to show debug information about all registers.**

### Screenshots

screenshots here
