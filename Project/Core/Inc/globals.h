#ifndef GLOBALS_H
#define GLOBALS_H
#include "gpio.h"

/*
* GLOBAL VARIABLES
*/
extern uint8_t buf[3];

extern uint8_t rightSwitch;
extern uint8_t leftSwitch;
extern uint8_t isCalibration;
extern uint8_t repeatCalibrationFlag;
extern float cartMovLength;
extern uint16_t count;
extern uint8_t flagBoarder;
extern int16_t speedPid;
extern float previousSpeed;
extern float prevSpeedErr;
extern float integral;
extern int32_t previousPosition;
extern int16_t speedCart;
extern int16_t privPosCart;
extern int16_t speedPole;
extern int16_t privPosPole;
extern int16_t cartPos;
extern int16_t polePos;
extern float cartSpeed;
extern float poleSpeed;
extern uint8_t rxBufferCounter;
extern float newCartSpeed;

/*
* ARRAYS
*/
extern uint8_t rxBuffer[100];
extern uint8_t dataForTransmitt[100];
extern uint8_t helpToTransmitt[1000];
extern uint8_t speedForTransmitt[8];
extern uint8_t posForTransmitt[4];
extern uint8_t rx_dt[100];

/*
* CONSTANTS
*/
extern const float SHIFT;
extern const int16_t BORDER;

extern int16_t speedCart_target;

#endif
