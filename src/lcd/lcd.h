#pragma once

#include "../../inc/stm32f103xb.h"

#include "../i2c/i2c.h"

#define LINE_LENGTH 16
#define LINE_OFFSET 16

void LCDinit();
void LCDclear();
void LCDchangeLine(uint8_t lineNumber);
void LCDprint(char *str, uint8_t length);
void lcdSendCommand(uint8_t command);
uint8_t lineLength(char *str);
