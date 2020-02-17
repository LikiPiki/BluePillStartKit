#pragma once

#include "../../inc/stm32f103xb.h"

#include "../i2c/i2c.h"

#define LINE_LENGTH 16
#define LINE_OFFSET 16

#define LCD_STATE_ON 1
#define LCD_STATE_OFF 0

#define ARROW_RIGHT 'z' + 4
#define ARROW_LEFT 'z' + 5

void LCDinit();
void LCDclear();
void LCDChangeLight(uint8_t lcdLightStatus);
void LCDchangeLine(uint8_t lineNumber);
void LCDprint(char *str, uint8_t length);
void lcdSendCommand(uint8_t command);
uint8_t lineLength(char *str);
