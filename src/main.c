#include <stdint.h>
#include <strings.h>

#include "../inc/stm32f103xb.h"

int seconds = 0;

void sendStringToPcf(char *str, uint8_t len) {
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR = (0x27 << 1);
	while (!(I2C1->SR1 & I2C_SR1_ADDR_Msk));
	I2C1->SR2;

	uint8_t one, two;
	for (uint8_t i = 0; i < len; i++) {
		one = str[i] & 0xF;
		two = str[i] & 0xF0;

		// sending bytes
		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = two | 0xD;
		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = two | 0x9;

		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = one << 4 | 0xD;

		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = one << 4 | 0x9;
	}

	while (!(I2C1->SR1 & I2C_SR1_BTF_Msk));
	I2C1->CR1 |= I2C_CR1_STOP;
}

void changePos(uint8_t pos) {
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR = (0x27 << 1);
	while (!(I2C1->SR1 & I2C_SR1_ADDR_Msk));
	I2C1->SR2;	

	if (pos == 1) {

		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = 0x80 | 0xC;
		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = 0x80 | 0x8;

		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = 0x0 | 0xC;
		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = 0x0 | 0x8;

	} else { // line 2
		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = 0xC0 | 0xC;
		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = 0xC0 | 0x8;

		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = 0x00 | 0xC;
		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = 0x00 | 0x8;
	}
	while (!(I2C1->SR1 & I2C_SR1_BTF_Msk));
	I2C1->CR1 |= I2C_CR1_STOP;

	for (uint32_t i = 0; i < 10000; i++);
}

void ADCInit() {
	// analog a1
	GPIOA->CRL &= ~GPIO_CRL_MODE1_Msk;

	// enable adc rcc
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	// on swstart
	ADC1->CR2 |= ADC_CR2_EXTTRIG;

	// // single conversation mode
	// ADC1->CR2 |= ADC_CR2_CONT;
	// // use wstart
	ADC1->CR2 |= ADC_CR2_EXTSEL;

	// 1channel selected
	// ADC1->CR1 |= 0x1;

	ADC1->SQR3 |= 1;

	// on adc
	ADC1->CR2 |= ADC_CR2_ADON;

	ADC1->CR2 |= ADC_CR2_RSTCAL;
	while (ADC1->CR2 & ADC_CR2_RSTCAL);

	ADC1->CR2 |= ADC_CR2_CAL;
	while (ADC1->CR2 & ADC_CR2_CAL);	
}

int main() {

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

	// i2c pb6, pb7 alternative funcs open drain
	GPIOB->CRL |= (0xFF << 24);

	// enable i2c
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	seconds++;

	I2C1->CR2 |= 8;
	I2C1->TRISE |= 8 + 1;
	I2C1->CCR |= 0x28;
	// i2c enable
	I2C1->CR1 |= I2C_CR1_PE;
	printf("debug here");

	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR = (0x27 << 1);
	while (!(I2C1->SR1 & I2C_SR1_ADDR_Msk));
	
	//reading SR2
	I2C1->SR2;

	// pcf settings array
	uint8_t bytes1[16] = {0x3C, 0x38, 0x3C, 0x38, 0x3C, 0x38, 0x2C, 0x28, 0x2C, 0x28, 0x8C, 0x88, 0x0C, 0x08, 0xDC, 0xD8};
	for (uint8_t i = 0; i < 16; i++) {
		while(!(I2C1->SR1 & I2C_SR1_TXE_Msk));
		I2C1->DR = bytes1[i];
	}

	while (!(I2C1->SR1 & I2C_SR1_BTF_Msk));
	I2C1->CR1 |= I2C_CR1_STOP;

	for (uint32_t i = 0; i < 10000; i++);

	ADCInit();

	while (1) {
		for	(int i = 0; i < 200000; i++);
		changePos(2);
		// start conversation on ADC
		ADC1->CR2 |= ADC_CR2_SWSTART;		
		while (!(ADC1->SR & ADC_SR_EOC));
		uint16_t data = ADC1->DR & ADC_DR_DATA_Msk;

		char str[10];
		sprintf(str, "DH %4d  \0", data);

		sendStringToPcf(str, 7);
	}

	return 0;
}
