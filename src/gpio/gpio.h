#pragma once

#include "../../inc/stm32f103xb.h"

#define STATE_ON 1
#define STATE_OFF 0

typedef enum {
    PIN8 = 8,
    PIN9,
} pin;

void gpioInit();

void mosfetSetState(pin p, uint8_t state);
void led2SetState(uint8_t state);
void led3SetState(uint8_t state);
