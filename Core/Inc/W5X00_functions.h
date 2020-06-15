
/*
 * W5X00_functions.h
 *
 *  Created on: May 4, 2020
 *      Author: BER83WI
 */

#ifndef INC_W5X00_FUNCTIONS_H_
#define INC_W5X00_FUNCTIONS_H_

#include "pin.h"

void setW5X00Spi(SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t);
void setW5X00UART(UART_HandleTypeDef*);
void W5X00_init(uint8_t*); //, pin_t);

uint8_t W5X00_open_wait(uint32_t);

void W5X00cs_sel();
void W5X00cs_desel();
uint8_t W5X00spi_rb(void);
void W5X00spi_wb(uint8_t);

__weak uint8_t W5X00_send_Status(uint8_t);
void createStatus(char []);

#endif /* INC_W5X00_FUNCTIONS_H_ */
