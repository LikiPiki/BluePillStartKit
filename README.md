# STM32 Blue pill empty project

## Installing
Installing dependencies. Also you need install VSCode.
For Mac:
```
brew install arm-none-eabi-gcc stlink
```

For Linux (arch):
```
sudo pacman -S arm-none-eabi-gcc stlink arm-none-eabi-gdb
```
arm-none-eabi-gdb for debbuging

Installing Vscode plugins
- Cortex debug
- ARM
- C/C++

## Building project

```
make - make and flash stm32 blue pill
make production - make without -gddb flag
make flash - flash only
```

### Debug with VSCode (Cortex debug plugin)
All debugger congigurations you can find in `.vscode/launch.json`

**You also need .SVD file to you microcontroller , to show debug information about all registers.**

### Screenshots

screenshots here
