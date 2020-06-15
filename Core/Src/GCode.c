#include "GCode.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

#define DTZ '.'
#define BUFFER_SIZE 16

GCodePara_t GCodeBuffer[BUFFER_SIZE];
int8_t curPara = 0;
int8_t newPara = 0;
int8_t paraCount = 0;

GCodePara_t parseString(uint8_t *origstring) {
	char *string = (char*)origstring;
	GCodePara_t temppara;
	temppara.valid = 0;
	temppara.validI = 0;
	temppara.validJ = 0;
	temppara.validK = 0;
	temppara.validP = 0;
	temppara.validS = 0;
	temppara.validO = 0;
	temppara.validX = 0;
	temppara.validY = 0;
	temppara.validZ = 0;
	temppara.validE = 0;
	temppara.validF = 0;

	uint8_t i = 0;

	if(string[i] == 'G' || string[i] == 'g') {
		temppara.character = 'G';
	} else if (string[i] == 'M' || string[i] == 'm') {
		temppara.character = 'M';
	} else {
		return temppara;
	}

	i++;
	if((temppara.valid = isdigit(string[i])) > 0) {
		temppara.number = (string[i] - 48);
	}

	i++;
	while(isdigit(string[i])) {
		temppara.number = (temppara.number * 10) + (string[i] - 48);
		i++;
	}

	i++;
	for (; i < strlen(string); i++) {
		if(isalpha(string[i])) {
			int valid = 0;
			if(string[i] == 'I' || string[i] == 'i') {
				valid = temppara.validI = findParaInt(string, i+1, &(temppara.I));
			} else if(string[i] == 'J' || string[i] == 'j') {
				valid = temppara.validJ = findParaInt(string, i+1, &(temppara.J));
			} else if(string[i] == 'K' || string[i] == 'k') {
				valid = temppara.validK = findParaInt(string, i+1, &(temppara.K));
			} else if(string[i] == 'P' || string[i] == 'p') {
				valid = temppara.validP = findParaInt(string, i+1, &(temppara.P));
			} else if(string[i] == 'S' || string[i] == 's') {
				valid = temppara.validS = findParaInt(string, i+1, &(temppara.S));
			} else if(string[i] == 'O' || string[i] == 'o') {
				valid = temppara.validO = findParaFloat(string, i+1, &(temppara.O));
			} else if(string[i] == 'X' || string[i] == 'x') {
				valid = temppara.validX = findParaFloat(string, i+1, &(temppara.X));
			} else if(string[i] == 'Y' || string[i] == 'y') {
				valid = temppara.validY = findParaFloat(string, i+1, &(temppara.Y));
			} else if(string[i] == 'Z' || string[i] == 'z') {
				valid = temppara.validZ = findParaFloat(string, i+1, &(temppara.Z));
			} else if(string[i] == 'E' || string[i] == 'e') {
				valid = temppara.validE = findParaFloat(string, i+1, &(temppara.E));
			} else if(string[i] == 'F' || string[i] == 'f') {
				valid = temppara.validF = findParaInt(string, i+1, &(temppara.F));
			} else {
//				USBD_CDC_TransmitPacket(pdev);
			}
			i += valid;
		}
	}

	return temppara;
}

uint8_t findParaInt (char *string, uint8_t i, int32_t *para) {

	int sign = 1;
	if(string[i] == '-') {
		i++;
		if (i >= strlen(string)) {
			return 0;
		}
		sign = -1;
	} else if(!isdigit(string[i])) {
		return 0;
	}

	int tempint = string[i] - 48;
	i++;
	while((i < strlen(string)) && isdigit(string[i])) {
		tempint = (tempint * 10) + (string[i] - 48);
		i++;
	}
	*para = tempint * sign;

	return 1;
}

float findParaFloat (char *string, int pos, float *para) {
//	char *string = (char*)origstring;
	int sign = 1;
	if(string[pos] == '-') {
		pos++;
		if (pos >= strlen(string)) {
			return 0;
		}
		sign = -1;
	} else if(!isdigit(string[pos])) {
		return 0;
	}

	unsigned int j = 1;
	unsigned int tempint = 0;
	int nachkomma = 0;
	while(isdigit(string[pos]) || (string[pos] == DTZ)) {
		if(string[pos] == DTZ) {
			if(nachkomma == 1) {
				return 0;
			} else {
				nachkomma = 1;
			}
		} else {
			tempint = (tempint * 10) + (string[pos] - 48);
			if(nachkomma) {
				j *= 10;
			}
		}
		pos++;
	}

	*para = ((float)tempint) / j * sign;

	return 1;
}

void printString(GCodePara_t para, uint8_t *origstring) {
	char *string = (char*)origstring;
	char tempstring[200] = "";
	char temp[20] = "";

	if(para.valid) {
		strcat(tempstring, "It is GCode with the following parameters:\n");
		sprintf(temp, "%c%d\n", para.character, para.number);
		strcat(tempstring, temp);
	} else {
		strcat(tempstring, "The GCode was NOT valid.");
		return;
	}

	if(para.validI) appendIntPara(tempstring, 'I', para.I);
	if(para.validJ) appendIntPara(tempstring, 'J', para.J);
	if(para.validK) appendIntPara(tempstring, 'K', para.K);
	if(para.validP) appendIntPara(tempstring, 'P', para.P);
	if(para.validS) appendIntPara(tempstring, 'S', para.S);

	if(para.validO) appendFloatPara(tempstring, 'O', para.O);
	if(para.validX) appendFloatPara(tempstring, 'X', para.X);
	if(para.validY) appendFloatPara(tempstring, 'Y', para.Y);
	if(para.validZ) appendFloatPara(tempstring, 'Z', para.Z);
	if(para.validE) appendFloatPara(tempstring, 'E', para.E);

	strcpy(string, tempstring);
}

void appendIntPara(char *string, char toappend, int32_t para) {
	char temp[20] = "";

	sprintf(temp, "%c=%d\n", toappend, (int)para);
	strcat(string, temp);
}

void appendFloatPara(char *string, char toappend, float para) {
	char temp[20] = "";

	sprintf(temp, "%c=%f\n", toappend, para);
	strcat(string, temp);
}

uint8_t addParaGC(GCodePara_t para) {
	if(isFullGC()) {
		return 0;
	}

	if(isEmergencyPara(&para)) {
// ALARM ... ALARM !!!
	} else if(isQueueClearPara(&para)) {
		clearQueueGC();
	} else {
		GCodeBuffer[newPara] = para;
		newPara++;
		if(newPara > (BUFFER_SIZE - 1)) {
			newPara = 0;
		}
		paraCount++;
	}

	return 1;
}

uint8_t getParaGC(GCodePara_t *temppara) {
	if(paraCount == 0) {
		return 0;
	}

	int tempPos = curPara;
	curPara++;
	if(curPara > (BUFFER_SIZE - 1)) {
		curPara = 0;
	}
	if(paraCount > 0) paraCount--;

	*temppara = GCodeBuffer[tempPos];
	return 1;
}

uint8_t isEmptyGC() {
	if(paraCount == 0) {
		return 1;
	}
	return 0;
}

uint8_t isFullGC() {
	return (paraCount == (BUFFER_SIZE - 1));
}

uint8_t clearQueueGC() {
	if(isEmptyGC()) {
		return 0;
	}
	curPara = 0;
	newPara = 0;
	paraCount = 0;

	return 1;
}

uint8_t isMovementPara(GCodePara_t *para) {
	if(para->character == 'G') {
		return 1;
	}
	return 0;
}

uint8_t isNextMovementPara() {
	if(!isEmptyGC() && isMovementPara(&GCodeBuffer[curPara])) {
		return 1;
	}
	return 0;
}

uint8_t isImmediatePara(GCodePara_t *para) {

	return 0;
}

uint8_t isEmergencyPara(GCodePara_t *para) {
	return 0;
}

uint8_t isQueueClearPara(GCodePara_t *para) {
	return 0;
}

uint8_t getMovementCom(GCodePara_t *temppara) {
	if(isNextMovementPara()) {
		return getParaGC(temppara);
	}
	return 0;
}

void exeNonMovePara() {

}

uint8_t MovementDone(GCodePara_t *temppara) {
	while(!isEmptyGC()) {
		if(isNextMovementPara()) {
			getParaGC(temppara);
			return 1;
		} else {
			exeNonMovePara();
		}
	}
	return 0;
}

void copyGCode(GCodePara_t *origpara, GCodePara_t *newpara) {
	if(!(origpara->valid = newpara->valid)) {
		return;
	}
	if((origpara->validI = newpara->validI)) {
		origpara->I = newpara->I;
	}
	if((origpara->validJ = newpara->validJ)) {
		origpara->J = newpara->J;
	}
	if((origpara->validK = newpara->validK)) {
		origpara->K = newpara->K;
	}
	if((origpara->validP = newpara->validP)) {
		origpara->P = newpara->P;
	}
	if((origpara->validS = newpara->validS)) {
		origpara->S = newpara->S;
	}
	if((origpara->validO = newpara->validO)) {
		origpara->O = newpara->O;
	}
	if((origpara->validX = newpara->validX)) {
		origpara->X = newpara->X;
	}
	if((origpara->validY = newpara->validY)) {
		origpara->Y = newpara->Y;
	}
	if((origpara->validZ = newpara->validZ)) {
		origpara->Z = newpara->Z;
	}
	if((origpara->validE = newpara->validE)) {
		origpara->E = newpara->E;
	}
	if((origpara->validF = newpara->validF)) {
		origpara->F = newpara->F;
	}
}
