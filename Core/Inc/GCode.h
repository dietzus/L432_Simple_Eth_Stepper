/*
 * GCode.h
 *
 *  Created on: Dec 8, 2019
 *      Author: Martin
 */

#ifndef INC_GCODE_H_
#define INC_GCODE_H_

#include <stdint.h>

typedef struct GCodePara {
	uint8_t valid;
	char character;
	uint8_t number;
	uint8_t validP;
	int32_t P;
	uint8_t validS;
	int32_t S;
	uint8_t validI;
	int32_t I;
	uint8_t validJ;
	int32_t J;
	uint8_t validK;
	int32_t K;
	uint8_t validO;
	float O;
	uint8_t validX;
	float X;
	uint8_t validY;
	float Y;
	uint8_t validZ;
	float Z;
	uint8_t validE;
	float E;
	uint8_t validF;
	int32_t F;
} GCodePara_t;

GCodePara_t parseString(uint8_t *);
void printString(GCodePara_t, uint8_t *);
uint8_t findParaInt (char *, uint8_t, int32_t *);
float findParaFloat (char *, int, float *);
void appendIntPara(char *, char, int32_t);
void appendFloatPara(char *, char, float);
void copyGCode(GCodePara_t *, GCodePara_t *);

uint8_t addParaGC(GCodePara_t);
uint8_t getParaGC(GCodePara_t *);
uint8_t isEmptyGC();
uint8_t isFullGC();
uint8_t clearQueueGC();

uint8_t isMovementPara(GCodePara_t *);
uint8_t isNextMovementPara();
uint8_t isImmediatePara(GCodePara_t *);
uint8_t isEmergencyPara(GCodePara_t *);
uint8_t isQueueClearPara(GCodePara_t *);

uint8_t getMovementCom(GCodePara_t *);
void exeNonMovePara();
uint8_t MovementDone(GCodePara_t *);

#endif /* INC_GCODE_H_ */
