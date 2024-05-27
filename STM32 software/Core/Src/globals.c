#include "globals.h"

/*
* GLOBAL VARIABLES
*/
uint8_t buf[3];

uint8_t rightSwitch = 0;
uint8_t leftSwitch = 0;
uint8_t isCalibration = 0;
uint8_t repeatCalibrationFlag = 0;
uint16_t count = 0;
float cartMovLength = 0;
uint8_t flagBoarder = 0;
int16_t speedPid = 0;
float previousSpeed = 0;
float prevSpeedErr = 0;
float integral = 0;
int32_t previousPosition = 0;

int16_t speedCart = 0;
int16_t speedCart_target = 0;

int16_t privPosCart = 0;
int16_t speedPole = 0;
int16_t privPosPole = 0;
int16_t cartPos = 0;
int16_t polePos = 0;
float cartSpeed = 0;
float poleSpeed = 0;
float newCartSpeed = 0;
uint8_t rxBufferCounter = 0;
/*
* ARRAYS
*/
//uint8_t rxBuffer[100] = {0};
//uint8_t dataForTransmitt[100] = {0};
//uint8_t helpToTransmitt[1000] = {0};
//uint8_t speedForTransmitt[8] = {0};
//uint8_t posForTransmitt[4] = {0};
//uint8_t rx_dt[100] = {0};

/*
* CONSTANTS
*/
const float SHIFT = 32000;
const int16_t BORDER = 6500;
