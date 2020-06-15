/*
 * Utils.h
 *
 *  Created on: 21.01.2020
 *      Author: Martin
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"

uint8_t DWT_Delay_Init(void);

uint32_t DWT_get_us(void);
void DWT_Delay_us(volatile uint32_t);

#endif /* INC_UTILS_H_ */
