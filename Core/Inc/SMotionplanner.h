/*
 * Motionplanner.h
 *
 *  Created on: Jun 2, 2020
 *      Author: Martin
 */

#ifndef INC_SMOTIONPLANNER_H_
#define INC_SMOTIONPLANNER_H_

#include "main.h"
#include "GCode.h"
#include "pin.h"

#define NRAXIS 6

extern TIM_HandleTypeDef htim1;

const static uint8_t SPMM[NRAXIS] = { 30, 30, 30, 30, 30, 30 };
const static uint16_t MINV[NRAXIS] = { 1500, 1500, 1500, 1500, 1500, 1500 };
const static uint16_t MAXV[NRAXIS] = { 50, 50, 50, 50, 50, 50 };
const static uint8_t a[NRAXIS] = { 10, 10, 10, 10, 10, 10 };
const static uint8_t minHighTime[NRAXIS] = { 20, 20, 20, 20, 20, 20 };

uint8_t initDefaultPins(uint8_t);
uint8_t initPins(pin_t*[], pin_t*[], pin_t*[], uint8_t);

uint8_t calcSMove(GCodePara_t);
uint8_t parsePara(GCodePara_t, uint8_t);
uint8_t parseGpara(GCodePara_t);
uint8_t parseMpara(GCodePara_t);

uint8_t getRamps(uint8_t);

void steptick();
void home(GCodePara_t, uint8_t);

void StepHigh(uint8_t);
void StepLow(uint8_t);
void SetDirFor(uint8_t);
void SetDirRev(uint8_t);
void SetEn(uint8_t);
void SetDis(uint8_t);

#endif /* INC_SMOTIONPLANNER_H_ */
