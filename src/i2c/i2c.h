#pragma once

#include "../../inc/stm32f103xb.h"

void I2Cinit();
void I2C1_EV_IRQHandler();

void I2Csend(uint8_t addr, uint8_t *data, uint8_t len);
void I2Cread(uint8_t addr, uint8_t readAddr, uint8_t *data, uint8_t len);