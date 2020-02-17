#include "gpio.h"

void gpioInit() {
    // led init
	//LED3 - red led
	GPIOA->CRH &= ~GPIO_CRH_MODE12;
	GPIOA->CRH &= ~GPIO_CRH_CNF12;
	GPIOA->CRH |= GPIO_CRH_MODE12;
	GPIOA->ODR |= GPIO_ODR_ODR12;

	// LED2 - green led
	GPIOB->CRL &= ~GPIO_CRL_CNF0;
	GPIOB->CRL &= ~GPIO_CRL_MODE0;
	GPIOB->CRL |= GPIO_CRL_MODE0;

    // set off state to mosfets by default
	// mosfet 1 pin A8
	GPIOA->CRH &= ~GPIO_CRH_MODE8;
	GPIOA->CRH &= ~GPIO_CRH_CNF8;
	GPIOA->CRH |= GPIO_CRH_MODE8;
	GPIOA->ODR |= GPIO_ODR_ODR8;

	// mosfet 1 pin A9
	GPIOA->CRH &= ~GPIO_CRH_MODE9;
	GPIOA->CRH &= ~GPIO_CRH_CNF9;
	GPIOA->CRH |= GPIO_CRH_MODE9;
	GPIOA->ODR |= GPIO_ODR_ODR9;

}

void mosfetSetState(pin p, uint8_t state) {
    if (state) {
        GPIOA->ODR |= 1 << p;
    } else {
		GPIOA->ODR &= ~(1 << p);
    }
}

void led2SetState(uint8_t state) {
    if (state) {
		GPIOB->ODR |= GPIO_ODR_ODR0;
    } else {
		GPIOB->ODR &= ~GPIO_ODR_ODR0;
    }
}

void led3SetState(uint8_t state) {
    if (state) {
		GPIOA->ODR |= GPIO_ODR_ODR12;
    } else {
		GPIOA->ODR &= ~GPIO_ODR_ODR12;
    }
}