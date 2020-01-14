#include "i2c.h"

// interupts
void I2C1_EV_IRQHandler() {
	if (I2C1->SR1 & I2C_SR1_SB) {
		// adres to write
		I2C1->DR = 0x27 << 1;
	} else if (I2C1->SR1 & I2C_SR1_ADDR) {
		I2C1->SR2;
	} else if (I2C1->SR1 & I2C_SR1_BTF) {
		I2C1->DR;
		// Off 6 channel DMA
		DMA1_Channel6->CCR &= ~DMA_CCR_EN;
		I2C1->CR1 |= I2C_CR1_STOP;
	}
}

void I2Cinit() {
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	
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

	//i2c setup
	I2C1->CR2 |= I2C_CR2_DMAEN;
	// speed 24MHz
	I2C1->CR2 |= I2C_CR2_FREQ_3 | I2C_CR2_FREQ_4;
	I2C1->CR2 |= I2C_CR2_ITEVTEN;

	// 100KHz
	I2C1->CCR |= 30;
	// 24 + 1
	I2C1->TRISE |= 25;
	I2C1->CR1 |= I2C_CR1_PE;

	DMA1_Channel6->CPAR = (uint32_t) &I2C1->DR;

	// DMA setup
	DMA1_Channel6->CCR |= DMA_CCR_MINC;
	DMA1_Channel6->CCR |= DMA_CCR_DIR;
}

void I2Csend(uint8_t *data, uint8_t len) {
	// wait DMA OFF and I2c Off
	while((DMA1_Channel6->CCR & DMA_CCR_EN) || (I2C1->CR1 & I2C_CR1_STOP));
	// 16 bytes to send (for lcd)
	DMA1_Channel6->CNDTR = len; 
	DMA1_Channel6->CMAR = (uint32_t) data;

	DMA1_Channel6->CCR |= DMA_CCR_EN;
	I2C1->CR1 |= I2C_CR1_START;
}
