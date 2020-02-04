#include <stdint.h>
#include <strings.h>
#include <stdio.h>

#include "../inc/stm32f103xb.h"

#include "i2c/i2c.h"
#include "lcd/lcd.h"

#define TRUE 1
#define FALSE 0

uint8_t i2c_k[4] = {'k' & 0xF0 | 0xD, 'k' & 0xF0 | 0x9, ('k' & 0xF) << 4 | 0xD, ('k' & 0xF) << 4 | 0x9};

const uint32_t fastDelay = 1e5;
const uint32_t slowDelay = 1e6;

const uint8_t HTU_ADDR = 0x40;
uint8_t dhtData[3] = {0, 0, 0};

uint32_t temp0 = 0;
uint32_t temp1 = 0;

uint32_t time = slowDelay;

char menu[2][16] = {
	"Temp: %2.d\0",
	"THT: \0",
};

void initRCC() {
	RCC->CIR |= RCC_CIR_HSERDYIE;
	RCC->CIR |= RCC_CIR_PLLRDYIE;

	RCC->CR |= RCC_CR_HSEON;

	NVIC_EnableIRQ(RCC_IRQn);

}

void RCC_IRQHandler() {
	if (RCC->CIR & RCC_CIR_HSERDYF) {
		RCC->CIR |= RCC_CIR_HSERDYC;
		// 8x3 = 24 MHz
		RCC->CFGR |= RCC_CFGR_PLLMULL3;
		RCC->CFGR |= RCC_CFGR_PLLSRC;
		// ADC prescaler 24/2 = 12MHz (14 MAX)
		RCC->CFGR |= RCC_CFGR_ADCPRE_DIV2;

		RCC->CR |= RCC_CR_PLLON;
	} else if (RCC->CIR & RCC_CIR_PLLRDYF) {
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		RCC->CIR |= RCC_CIR_PLLRDYC;
	}
}

void initADC() {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRL &= ~GPIO_CRL_MODE0;
	GPIOA->CRL &= ~GPIO_CRL_CNF0;

	GPIOA->CRL &= ~GPIO_CRL_MODE1;
	GPIOA->CRL &= ~GPIO_CRL_CNF1;

	ADC1->CR1 |= ADC_CR1_SCAN;
	// enabel intrupt on end ADC convertion
	ADC1->CR1 |= ADC_CR1_JEOCIE;
	ADC1->CR2 |= ADC_CR2_JEXTTRIG;
	// set 111 JSWSTART
	ADC1->CR2 |= ADC_CR2_JEXTSEL; 

	// 2 conversation
	ADC1->JSQR |= ADC_JSQR_JL_0;

	ADC1->JSQR |= ADC_JSQR_JSQ4_0;

	NVIC_EnableIRQ(ADC1_2_IRQn);

	// on ADC
	ADC1->CR2 |= ADC_CR2_ADON;

	// Сбрасываем калибровку
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while((ADC1->CR2 & ADC_CR2_RSTCAL) == ADC_CR2_RSTCAL);
	
    // Запускаем калибровку
    ADC1->CR2 |= ADC_CR2_CAL;
    while((ADC1->CR2 & ADC_CR2_CAL) != ADC_CR2_CAL);
}

void ADC1_2_IRQHandler() {
	temp0 = ADC1->JDR1;
	temp1 = ADC1->JDR2;
	ADC1->SR &= ~ADC_SR_JEOC;
}

void delay() {
	for (uint32_t i = 0; i < time; i++);
}

// Button RIGHT interrupt
void EXTI15_10_IRQHandler() {
	GPIOB->ODR ^= GPIO_ODR_ODR0;
	EXTI->PR |= EXTI_PR_PR15;
}

void buttonSetup() {
	// cloking B port
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PB;

	// button B15 RIGHT
	GPIOB->CRH &= ~GPIO_CRH_MODE15;
	GPIOB->CRH &= ~GPIO_CRH_CNF15_0;
	GPIOB->CRH |= GPIO_CRH_CNF15_1;

	GPIOB->ODR |= GPIO_ODR_ODR15;

	EXTI->FTSR |= EXTI_FTSR_TR15;
	EXTI->IMR |= EXTI_IMR_MR15;

	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void initLED() {
	// cloking C port
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	// LED C13
	GPIOB->CRL &= ~GPIO_CRL_CNF0;

	GPIOB->CRL &= ~GPIO_CRL_MODE0; 
	GPIOB->CRL |= GPIO_CRL_MODE0; 
}

int main() {
	delay(); // for display
	initRCC();
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));
	initLED();
	initADC();
	buttonSetup();

	// start adc
	ADC1->CR2 |= ADC_CR2_JSWSTART;

	I2Cinit();
	I2Cread(HTU_ADDR, 0xE3, dhtData, 3);
	uint16_t temp = dhtData[0] << 8 | dhtData[1];
	temp = (0.002681 * temp - 46.85);

	LCDinit();
	LCDclear();
	delay();
	sprintf(menu[0], "Temp: %2.d\0", temp);
	sprintf(menu[1], "H1:%3.d  H2:%3.d", temp0, temp1);
	LCDprint(menu[0], lineLength(menu[0]));
	delay();
	LCDchangeLine(1);
	LCDprint(menu[1], lineLength(menu[1]));

	while (TRUE) {

		GPIOB->ODR |= GPIO_ODR_ODR0;
		delay();
		GPIOB->ODR &= ~GPIO_ODR_ODR0;
 		delay();
	}

	return 0;
}