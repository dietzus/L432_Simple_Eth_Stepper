/*
 * Utils.c
 *
 *  Created on: 21.01.2020
 *      Author: Martin
 */

#include "Utils.h"

uint8_t usdiv = 1;

uint8_t DWT_Delay_Init(void) {
	SystemCoreClockUpdate();

// Disable TRC
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000
// Enable TRC
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000
//	Disable Clock Cycle Counter
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001
//	Enable Clock Cycle Counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //~0x00000001
//	Reset the clock cycle counter value
	DWT->CYCCNT = 0;
//	3 NOP Operations as a function test
	asm("NOP");
	asm("NOP");
	asm("NOP");
//	Check if the counter has actually started
	if(DWT->CYCCNT) {
		usdiv = HAL_RCC_GetHCLKFreq() / 1000000;
		return 0; //Clock cycle counter did start
	} else {
		return 1; //Clock cycle counter
	}
}

uint32_t DWT_get_us(void) {
	return DWT->CYCCNT / usdiv;
}

/**
 * @brief This function provides a delay (in microseconds)
 * @param microseconds: delay in microseconds
 */
void DWT_Delay_us(volatile uint32_t microseconds) {
	uint32_t clk_cycle_start = DWT_get_us();
	/* Delay till end */
	while ((DWT_get_us() - clk_cycle_start) < microseconds);
}
