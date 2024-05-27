#ifndef PENDULUM_LIB_H
#define PENDULUM_LIB_H
#include "pendulum_lib.h"
#include "usart.h"

#define LD4_Pin LL_GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin LL_GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin LL_GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin LL_GPIO_PIN_15
#define LD6_GPIO_Port GPIOD


//void transmitData(USART_TypeDef *USARTx, uint8_t *data, int length);
void moveCarridge(int32_t speed);
void start_melody(void);
int8_t poleCalibration(void);
uint8_t calibration(void);
int16_t getTicksCartPosition(void);
int16_t getTicksPolePosition(void);
float getCartSpeed(void);
float getPoleSpeed(void);
uint8_t moveToPosition(float setpoint);
void speedController(void);
//uint8_t* readData(USART_TypeDef *USARTx);
//void terminal(uint8_t instruction);
//long double getMetersCartPosition(void);
//long double getRadianPolePosition(void);
#endif
