/*
 * Terminal.cpp
 *
 *  Created on: Feb 2, 2024
 *      Author: Ruslan
 */

#include "Terminal.h"


uint8_t str[25];
//uint8_t buf[3];
uint8_t rsp[25];
uint8_t crc = 0;

struct state_t state;
struct state_time_t state_time;

uint8_t CMD_LNG_IN[Count] =  {0, 4, 0,  0, 0};
uint8_t CMD_LNG_OUT[Count] = {0, 0, 12, 0, 14};

extern UART_HandleTypeDef huart5;

uint8_t calc_crc(uint8_t cmd, uint8_t* data, uint8_t data_length){
	crc = cmd;
	for (uint8_t i = 0; i < data_length; i++){
		crc += data[i];
	}
	return crc;
}

HAL_StatusTypeDef Send_response(uint8_t cmd, uint8_t* data){
	uint8_t data_length = CMD_LNG_OUT[cmd];
	if (data_length == 0){
		rsp[0] = cmd;
		rsp[data_length+1] = cmd;
	} else {
		rsp[0] = cmd;
		rsp[data_length+1] = calc_crc(cmd, data, data_length);
		memcpy((uint8_t*)&rsp+1, data, data_length);
	}
	return (HAL_UART_Transmit(&huart5, rsp, data_length+2, data_length+2));
}

void TERMINAL(uint8_t cmd, uint8_t* str){

	switch (cmd){
	case CALIBRATE:
		{
		Send_response(CALIBRATE, (uint8_t*)&rsp);
		calibration();
		}
		break;

	case SET_SPEED:
		{
		Send_response(SET_SPEED, (uint8_t*)&rsp);
		int32_t speed;
		memcpy((uint8_t*)&speed, str, 4);
//		speedCart_target = speed;
		moveCarridge(speed);
		}
		break;

	case GET_STATE:
		{
		state.cart_pose = getTicksCartPosition();
		state.pole_pose = getTicksPolePosition();
		state.cart_speed = getCartSpeed();
		state.pole_speed = getPoleSpeed();
		Send_response(GET_STATE, (uint8_t*)&state);
		}
		break;

	case PING:
		{
		Send_response(PING, rsp);
		}
		break;

	case GET_STATE_TIME:
		{
		state_time.cart_pose = getTicksCartPosition();
		state_time.pole_pose = getTicksPolePosition();
		state_time.cart_speed = getCartSpeed();
		state_time.pole_speed = getPoleSpeed();
		state_time.time = HAL_GetTick();
		Send_response(GET_STATE_TIME, (uint8_t*)&state);
		}
		break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (buf[0] == '!'){
		if (HAL_UART_Receive(huart, buf+1, 1, 1) == HAL_OK && buf[1] == '!'){
			if ((HAL_UART_Receive(huart, buf+2, 1, 1) == HAL_OK) && (buf[2] < Count)){

				uint8_t cmd = buf[2];
				uint8_t cmd_lenght = CMD_LNG_IN[cmd];
				if (HAL_UART_Receive(huart, str, cmd_lenght+1, cmd_lenght+1) == HAL_OK){

					uint8_t crc_recieved = str[cmd_lenght];
					crc = calc_crc(cmd, str, cmd_lenght);
					if (crc == crc_recieved){

						TERMINAL(cmd, str);
					}
				}
			}
		}
	}
	HAL_UART_Receive_IT (huart, buf, 1);
}
