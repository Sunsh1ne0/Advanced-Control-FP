/*
 * Terminal.h
 *
 *  Created on: Jan 20, 2024
 *      Author: Ruslan
 */

#ifndef INC_TERMINAL_H_
#define INC_TERMINAL_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <string.h>

#include "globals.h"
#include "pendulum_lib.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

enum CMD {CALIBRATE,
		SET_SPEED,
		GET_STATE,
		PING,
		GET_STATE_TIME,
		Count };

struct __attribute((packed)) state_t{
	int16_t pole_pose;
	int16_t cart_pose;
	float pole_speed;
	float cart_speed;
};

struct __attribute((packed)) state_time_t{
	int16_t pole_pose;
	int16_t cart_pose;
	float pole_speed;
	float cart_speed;
	int16_t time;
};


//extern uint8_t buf[3];

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

uint8_t calc_crc(uint8_t cmd, uint8_t* data, uint8_t data_length);
HAL_StatusTypeDef Send_response(uint8_t cmd, uint8_t* data);
void TERMINAL(uint8_t cmd, uint8_t* str);
//
///* USER CODE END EFP */
//
///* Private defines -----------------------------------------------------------*/
//
///* USER CODE BEGIN Private defines */
//
///* USER CODE END Private defines */


#ifdef __cplusplus
}
#endif

#endif /* INC_TERMINAL_H_ */
