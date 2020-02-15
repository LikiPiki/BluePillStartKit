#pragma once

#include "../../inc/stm32f103xb.h"

#define STATE_ON 1
#define STATE_OFF 0

void gpioInit();

void mosfet1SetState(uint8_t state);
void mosfet2SetState(uint8_t state);
void led2SetState(uint8_t state);
void led3SetState(uint8_t state);
