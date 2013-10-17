#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
//Board = Arduino Mega 2560 or Mega ADK
#define __AVR_ATmega2560__
#define 
#define _VMDEBUG 1
#define ARDUINO 105
#define ARDUINO_MAIN
#define __AVR__
#define F_CPU 16000000L
#define __cplusplus
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__

#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
            
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}

bool checkBlockCMD();
bool checkTurnCMD();
void queueErrorMessage();
void queueBlockCmd();
void queueTurnoutCmd();
void serialEvent();
void serialEvent2();
void parseSerialCommand(int num1, int num2, int num3);
char* readSerial(int serNum);
int* getNumberCmds(char* stream);
int* readSerialNumeric(int serNum);
void readSerialCommand();
void setBlockPower(int blockNum, int powerLevel);
void setBlockPolarity(int blockNum, bool isCCW);
void updateBlocks(int i);
void doBlockCmds();
void doTurnoutCmds();
void updateTurnouts();
void checkTurnout();
void turnoutStates();
void setTurnStateBuffer(int turnoutNum);
void setBlockStateBuffer(int blockNum);
void readSensCaller();
void readSensors();
void sensorHitFunction();
void printSerial();
void writeShield(int bank, int pin, int pwr);
//
//

#include "H:\arduinoIDE\hardware\arduino\variants\mega\pins_arduino.h" 
#include "H:\arduinoIDE\hardware\arduino\cores\arduino\arduino.h"
#include "H:\Arduino\ArduinoTrainControl2\ArduinoTrainControl2.ino"
#include "H:\Arduino\ArduinoTrainControl2\OutDel.cpp"
#include "H:\Arduino\ArduinoTrainControl2\OutDel.h"
#include "H:\Arduino\ArduinoTrainControl2\Turnout.cpp"
#include "H:\Arduino\ArduinoTrainControl2\Turnout.h"
#endif
