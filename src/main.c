#include <stdint.h>
#include <strings.h>
#include <stdio.h>
#include <stdbool.h>

#include "../inc/stm32f103xb.h"

#include "gpio/gpio.h"
#include "i2c/i2c.h"
#include "lcd/lcd.h"

#define TRUE 1
#define FALSE 0

#define RECIEVE_BYTES_LEN 3

uint8_t i2c_k[4] = {'k' & 0xF0 | 0xD, 'k' & 0xF0 | 0x9, ('k' & 0xF) << 4 | 0xD, ('k' & 0xF) << 4 | 0x9};

const uint32_t fastDelay = 1e5;
const uint32_t slowDelay = 1e6;

const uint8_t HTU_ADDR = 0x40;
uint8_t dhtData[3] = {0, 0, 0};

uint32_t time = slowDelay;
bool startSending = false;
uint8_t someBytes[] = "T%03d";
uint8_t recievingBytesUSART3[3];

typedef struct {
	// Temperature
	uint16_t temp;
	uint8_t tempFrac;
	// Humidity 
	uint16_t h;
	uint8_t hFrac;

	uint8_t h0;
	uint8_t h0Frac;

	uint8_t h1;
	uint8_t h1Frac;
} sensors; 
sensors sens;

typedef struct {
	uint8_t first;
	uint8_t second;
} ipAdress;
ipAdress ip = {0, 0};

#define SLEEPS_CYCLES 3 // Timer counts to off display
typedef struct {
	uint8_t lcdLight;
	uint8_t counter;
} lcdController;
volatile lcdController lcd = {
	1, // Ligth on 1/0 - OFF
	0, // Counter default state
};

char menu[2][16];

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
	NVIC_SetPriority(ADC1_2_IRQn, 0xE0);

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
	sens.h0 = ((float)(4095 - ADC1->JDR1) / 4095) * 100;
	sens.h0Frac = (int) (((float)(4095 - ADC1->JDR1) / 4095) * 10000) % 100;
	sens.h1 = ((float)(4095 - ADC1->JDR2) / 4095) * 100;
	sens.h1Frac = (int) (((float)(4095 - ADC1->JDR2) / 4095) * 10000) % 100;

	ADC1->SR &= ~ADC_SR_JEOC;
}

void delay() {
	for (uint32_t i = 0; i < time; i++);
}

// Button Right interrupt
void EXTI15_10_IRQHandler() {
	if (lcd.lcdLight == 0) {
		lcd.lcdLight = 1;
		lcd.counter = 0;

		TIM2->EGR |= TIM_EGR_UG;
	}
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
	NVIC_SetPriority(EXTI15_10_IRQn, 0xE0);
}

void initTimer() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0xF0);

	TIM2->PSC = 23999;
	TIM2->ARR = 29999; // 30 seconds
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->SR &= ~TIM_SR_UIF;
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
}

void TIM2_IRQHandler() {
	ADC1->CR2 |= ADC_CR2_JSWSTART;
	// update sensors information
	I2Cread(HTU_ADDR, 0xE3, dhtData, 3);
	sens.temp = dhtData[0] << 8 | dhtData[1];
	sens.tempFrac = (int) (0.2681 * sens.temp - 46.85) % 100;
	sens.temp = (0.002681 * sens.temp - 46.85);

	I2Cread(HTU_ADDR, 0xE5, dhtData, 3);
	sens.h = dhtData[0] << 8 | dhtData[1];
	sens.hFrac = (int) (0.1907 * sens.h - 6) % 100;
	sens.h = (0.001907 * sens.h - 6);
	if (sens.h > 100) {
		sens.h = 100;
		sens.hFrac = 0;
	}

	// if need show data in display
	if (lcd.lcdLight == 1) {
		GPIOB->ODR ^= GPIO_ODR_ODR0;
		LCDclear();
		delay();
		sprintf(menu[0], "Temp: %2.d IP%d.%d\0", sens.temp, ip.first, ip.second);
		sprintf(menu[1], "H1:%3.d%%  H2:%3.d%%", sens.h0,  sens.h1);
		LCDprint(menu[0], lineLength(menu[0]));
		LCDchangeLine(1);
		LCDprint(menu[1], lineLength(menu[1]));
	}

	if (lcd.counter >= SLEEPS_CYCLES) {
		lcd.lcdLight = LCD_STATE_OFF;
		LCDChangeLight(LCD_STATE_OFF);
	} else {
		lcd.counter++;
	}

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

	// USART 115200 boud rate
	USART3->BRR |= 208;

	// DMA enable
	USART3->CR3 |= USART_CR3_DMAT;
	USART3->CR3 |= USART_CR3_DMAR;

	USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	// for (uint8_t i = 0; i < 4; i++) {
	// 	while (!(USART3->SR & USART_SR_TXE));
	// 	USART3->DR = i;
	// }

}

void DMA1_Channel2_IRQHandler() {
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1->IFCR |= DMA_IFCR_CGIF2;

	// Check flags

	//while(!(USART1->SR & USART_SR_TC));
	//startSending = false;
}

void DMA1_Channel3_IRQHandler() {
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1->IFCR |= DMA_IFCR_CGIF3;

	// handle bytes from uart

	// Recieving GET to post data to server
	if ((recievingBytesUSART3[0] == 'G') && (recievingBytesUSART3[1] == 'E')) {
		led2SetState(STATE_ON);
		static char ar[100];
		sprintf(ar, "T%03d%02dH%03d%02dh%03d%02dV%03d%02dOOOOOO",
			sens.temp,
			sens.tempFrac,
			sens.h0,
			sens.h0Frac,
			sens.h1,
			sens.h1Frac,
			sens.h,
			sens.hFrac
		);
		sendUsart(ar, 30);
	}

	// ESP8266 sending server IP address
	if ((recievingBytesUSART3[0] == 'I')) {
		ip.first = recievingBytesUSART3[1];
		ip.second = recievingBytesUSART3[2];

		TIM2->EGR |= TIM_EGR_UG;
	}

	DMA1_Channel3->CMAR = (uintptr_t) recievingBytesUSART3;
	DMA1_Channel3->CNDTR = RECIEVE_BYTES_LEN;

	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void initDMA() {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	// Settings for transiever
	// middle priority level
	DMA1_Channel2->CCR |= DMA_CCR_PL_0;

	// from peripherial to memory
	DMA1_Channel2->CCR |= DMA_CCR_DIR;
	DMA1_Channel2->CCR |= DMA_CCR_MINC;

	DMA1_Channel2->CPAR = &USART3->DR;

	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	// enable trasfer complete interupt
	DMA1_Channel2->CCR |= DMA_CCR_TCIE;

	// Settings for reciever

	// from memory to peripherial
	DMA1_Channel3->CCR |= DMA_CCR_MINC;

	DMA1_Channel3->CPAR = &USART3->DR;

	// enable trasfer complete interupt
	DMA1_Channel3->CCR |= DMA_CCR_TCIE;

	DMA1_Channel3->CMAR = (uintptr_t) recievingBytesUSART3;
	DMA1_Channel3->CNDTR = RECIEVE_BYTES_LEN;

	DMA1_Channel3->CCR |= DMA_CCR_EN;
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

void sendUsart(uint8_t* array, uint8_t len) {
	DMA1_Channel2->CMAR = (uintptr_t) array;
	DMA1_Channel2->CNDTR = len;
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
	ADC1->CR2 |= ADC_CR2_JSWSTART;
	buttonSetup();

	initUsart();
	initDMA();

	// start adc

	I2Cinit();
	LCDinit();
 	LCDclear();
 	delay();

	// Update screen and sensors
	TIM2->EGR |= TIM_EGR_UG;

	while (TRUE) {
 		delay();
	}

	return 0;
}
