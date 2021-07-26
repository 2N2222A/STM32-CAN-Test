#include "Futaba_VFD.h"


void delay(unsigned long time){
    // for(time; time > 0; time--){
    //     Nop();
    // }
    HAL_Delay(time);
}

void shiftOut(unsigned char data){
    unsigned char i;
    for (i = 0; i < 8; i++){ 
        CLK(0);
        if((data >> i) & 0x01){			//shift out data LSB first
            SDO(1);
        }else{
            SDO(0);
        }
        CLK(1);
    }
}

void VFDCommand(unsigned char command){
    CS(0);
    shiftOut(command);
    CS(1);
    
}

void displayData(void){
    CS(0);
    shiftOut(0xE8);
    CS(1);
    
}

void setBrightness(unsigned char brightness){
    CS(0);
    shiftOut(0xE4);
    delay(5);
    shiftOut(brightness);
    CS(1);
    delay(5);
}

void initVFD(void){
    CS(0);
    shiftOut(0xE0);
    delay(5);
    shiftOut(0x07);
    CS(1);
    delay(5);
    setBrightness(0x20);
}

void VFDWriteCharacter(unsigned char pos, unsigned char character){
    CS(0);
    shiftOut(0x20 + pos);
    shiftOut(character);
    CS(1);
    displayData();
}

void VFDWriteString(unsigned char pos, char *characters){
    CS(0);
    shiftOut(0x20 + pos);
    while(*characters){
        shiftOut(*characters);
        characters++;
    }
    CS(1);
    displayData();
    
}

void clear(void){
    char space[] = "        ";
    VFDWriteString(0, space);
}



