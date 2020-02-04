#include "i2c.h"

void I2Cinit() {
	// cloking B port
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// B6, B7 alternative functions open drain
	GPIOB->CRL &= ~GPIO_CRL_MODE6;
	GPIOB->CRL &= ~GPIO_CRL_CNF6;
	GPIOB->CRL |= GPIO_CRL_CNF6;
	GPIOB->CRL |= GPIO_CRL_MODE6_1;

	GPIOB->CRL &= ~GPIO_CRL_MODE7;
	GPIOB->CRL &= ~GPIO_CRL_CNF7;
	GPIOB->CRL |= GPIO_CRL_CNF7;
	GPIOB->CRL |= GPIO_CRL_MODE7_1;

	// speed 24MHz
	I2C1->CR2 |= I2C_CR2_FREQ_3 | I2C_CR2_FREQ_4;

	// 100KHz
	I2C1->CCR |= 30;
	// 24 + 1
	I2C1->TRISE |= 25;
	I2C1->CR1 |= I2C_CR1_PE;
}

void I2Csend(uint8_t addr, uint8_t *data, uint8_t len) {
	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->SR1;
	I2C1->DR = addr << 1;

	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	I2C1->SR1; I2C1->SR2;

	for (uint8_t i = 0; i < len; i++) {
		while(!(I2C1->SR1 & I2C_SR1_TXE));
		I2C1->DR = data[i];
		while(!(I2C1->SR1 & I2C_SR1_BTF));
	}

	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2Cread(uint8_t addr, uint8_t readAddr, uint8_t *data, uint8_t len) {
	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->SR1;
	I2C1->DR = addr << 1;

	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	I2C1->SR1; I2C1->SR2;

	while(!(I2C1->SR1 & I2C_SR1_TXE));
	I2C1->DR = readAddr;
	while(!(I2C1->SR1 & I2C_SR1_BTF));

	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->SR1;
	I2C1->DR = addr << 1 | 1;

	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	I2C1->SR1; I2C1->SR2;

	for (uint8_t i = 0; i < len; i++) {
		while(!(I2C1->SR1 & I2C_SR1_RXNE));
		data[i] = I2C1->DR;
		if (i == len - 2) {
			I2C1->CR1 &= ~I2C_CR1_ACK;
			I2C1->CR1 |= I2C_CR1_STOP;
		}

	}
}
