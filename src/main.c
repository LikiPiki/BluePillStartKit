#include <stdint.h>
#include <strings.h>
#include <stdio.h>

#include "../inc/stm32f103xb.h"

#include "gpio/gpio.h"
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
uint8_t someBytes[] = {'h', 'e', 'l', 'l', 'o'};

typedef struct {
	uint8_t displaySleepCounter;
	uint8_t displaySleepTime;
	uint8_t displaySleep;
} displayLightController;

const uint8_t TO_SLEEP_TIME = 2; // display sleeping time in minutes
displayLightController dispController = {0, TO_SLEEP_TIME, 0};

char menu[2][16] = {
	"Temp: %2.d\0",
	"THT: \0",
};

void initRCC() {
	// clocking by HSE crystal
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

	// Reset calibration
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while((ADC1->CR2 & ADC_CR2_RSTCAL) == ADC_CR2_RSTCAL);
	
    // Start calibration
    ADC1->CR2 |= ADC_CR2_CAL;
    while((ADC1->CR2 & ADC_CR2_CAL) != ADC_CR2_CAL);
}

void ADC1_2_IRQHandler() {
	temp0 = ((float)(4095 - ADC1->JDR1) / 4095) * 100;
	temp1 = ((float)(4095 - ADC1->JDR2) / 4095) * 100;
	ADC1->SR &= ~ADC_SR_JEOC;
}

void delay() {
	for (uint32_t i = 0; i < time; i++);
}

// Buttons interrupt
void EXTI15_10_IRQHandler() {
	// displaySleepCounter = 0;
	// if (displaySleep) {
	LCDChangeLight(STATE_ON);
		// displaySleep = 0;
	// }
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

void initTimer() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->PSC = 23999;
	TIM2->ARR = 59999; // 1 sec
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->SR &= ~TIM_SR_UIF;
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler() {
	dispController.displaySleepCounter++;
	if ((dispController.displaySleepCounter >= dispController.displaySleepTime) && (!dispController.displaySleep)) {
		LCDChangeLight(LCD_STATE_OFF);
		dispController.displaySleepCounter = 0;
		dispController.displaySleep = 1;
	}
	GPIOB->ODR ^= GPIO_ODR_ODR0;
	I2Cread(HTU_ADDR, 0xE3, dhtData, 3);
	uint16_t temp = dhtData[0] << 8 | dhtData[1];
	temp = (0.002681 * temp - 46.85);
	LCDclear();
	delay();
	sprintf(menu[0], "Temp: %2.d\0", temp);
	sprintf(menu[1], "H1:%3.d%%  H2:%3.d%%", temp0, temp1);
	LCDprint(menu[0], lineLength(menu[0]));
	LCDchangeLine(1);
	LCDprint(menu[1], lineLength(menu[1]));
	TIM2->SR &= ~TIM_SR_UIF;
}

void initUsart() {
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	// B10 alternative functions push-pull
	GPIOB->CRH &= ~GPIO_CRH_MODE10;
	GPIOB->CRH &= ~GPIO_CRH_CNF10;
	GPIOB->CRH |= GPIO_CRH_CNF10_1;
	GPIOB->CRH |= GPIO_CRH_MODE10;

	// B11 floating input pull-up
	GPIOB->CRH &= ~GPIO_CRH_MODE11;
	GPIOB->CRH &= ~GPIO_CRH_CNF11;
	GPIOB->CRH |= GPIO_CRH_CNF11_1;

	// (24 000 000 / 9 600) / 16 = 156,25 => 156 Mantisa
	// 0.25 * 16 = 4 Fraction

	// USART 9600 boud rate
	USART3->BRR |= (156 << 4) | 4;

	// DMA enable
	USART3->CR3 |= USART_CR3_DMAT;
	USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	// for (uint8_t i = 0; i < 3; i++) {
	// 	while (!(USART3->SR & USART_SR_TXE));
	// 	USART3->DR = i;
	// }
}

void DMA1_Channel2_IRQHandler() {
	led2SetState(STATE_ON);
	DMA1->IFCR |= DMA_IFCR_CGIF2;
}

void initDMA() {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	// middle priority level
	DMA1_Channel2->CCR |= DMA_CCR_PL_0;
	// from peripherial to memory
	DMA1_Channel2->CCR |= DMA_CCR_DIR;
	DMA1_Channel2->CCR |= DMA_CCR_MINC;

	DMA1_Channel2->CMAR = someBytes;
	DMA1_Channel2->CPAR = &USART3->DR;

	DMA1_Channel2->CNDTR = 5;

	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	// enable trasfer complete interupt
	DMA1_Channel2->CCR |= DMA_CCR_TCIE;

	DMA1_Channel2->CCR |= DMA_CCR_EN;
	USART3->SR &= ~USART_SR_TC;
}

int main() {
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

	initRCC();
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));

	initTimer();
	delay(); // for display
	gpioInit();
	initADC();
	buttonSetup();

	initUsart();
	initDMA();

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
	sprintf(menu[1], "H1:%3.d%%  H2:%3.d%%", temp0, temp1);
	LCDprint(menu[0], lineLength(menu[0]));
	LCDchangeLine(1);
	LCDprint(menu[1], lineLength(menu[1]));

	while (TRUE) {
 		delay();
	}

	return 0;
}
