#ifndef FUTABA_VFD_H
#define FUTABA_VFD_H

#include "main.h"
//#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"


#define CS(value)	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, value)
#define CLK(value)	HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, value)
#define SDO(value)	HAL_GPIO_WritePin(SDO_GPIO_Port, SDO_Pin, value)
#define RST(value)	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, value)


void delay(unsigned long time);
void shiftOut(unsigned char data);
void VFDCommand(unsigned char command);
void displayData(void);
void setBrightness(unsigned char brightness);
void initVFD(void);
void VFDWriteCharacter(unsigned char pos, unsigned char character);
void VFDWriteString(unsigned char pos, char *characters);
void clear(void);


#endif
