#include "lcd.h"

const uint8_t LCDAddr = 0x27;

const uint8_t i2c_init_bytes[16] = {0x3C, 0x38, 0x3C, 0x38, 0x3C, 0x38, 0x2C, 0x28, 0x2C, 0x28, 0x8C, 0x88, 0x0C, 0x08, 0xDC, 0xD8};

uint8_t i2c_lines_bytes[132];
uint8_t *i2c_line1 = i2c_lines_bytes;
uint8_t *i2c_line2 = i2c_lines_bytes + LINE_OFFSET;
uint8_t currentLine = 0;

void LCDinit() {
	I2Csend(LCDAddr, i2c_init_bytes, 16);
    currentLine = 0;
}

void lcdSendCommand(uint8_t command) {
    static int8_t command_bytes[4];
    command_bytes[0] = command | 0x0C;
    command_bytes[1] = command | 0x08;
    command_bytes[2] = command | 0x1C;
    command_bytes[3] = command | 0x18;
    I2Csend(LCDAddr, command_bytes, 4);
}

void LCDclear() {
    lcdSendCommand(0x00);
}

void LCDchangeLine(uint8_t lineNumber) {
    lcdSendCommand(0xB0);
}

void LCDprint(char *str, uint8_t length) {
    uint8_t *startLine = i2c_lines_bytes + currentLine * 68;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t currentInBytes = i * 4;
        *(startLine + currentInBytes) = str[i] & 0xF0 | 0xD;
        *(startLine + currentInBytes + 1) = str[i] & 0xF0 | 0x9;
        *(startLine + currentInBytes + 2) = (str[i] & 0xF) << 4 | 0xD;
        *(startLine + currentInBytes + 3) = (str[i] & 0xF) << 4 | 0x9;
    }
    I2Csend(LCDAddr, startLine, length * 4);
}

uint8_t lineLength(char *str) {
    uint8_t i = 0;
    for (; i < 15; i++) {
        if (str[i] == '\0') {
            return i;
        }
    }
    return i + 1;
}
