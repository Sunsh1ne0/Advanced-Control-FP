#include "pendulum_lib.h"
#include "globals.h"
#include "stdio.h"
#include "string.h"

float timeStep = 10000;

#define minspeed 0
#define maxspeed 600

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define constrain_sign(amt,low,high)((amt>0)?constrain(amt,low,high): constrain(amt,-high,-low))

extern float speed_coeff;
//void transmitData(USART_TypeDef *USARTx, uint8_t *data, int length)
//{
//	for (int i = 0; i < length; i++)
//	{
//		while (!LL_USART_IsActiveFlag_TXE(UART5));
//		LL_USART_TransmitData8(USARTx, data[i]);
//	}
//}

//uint8_t* readData(USART_TypeDef *USARTx)
//{
//	  uint16_t pos = 0;
//	  if(LL_USART_IsActiveFlag_RXNE(USARTx))
//	  {
//		  if (pos==0)
//		  {
//			  rx_dt[pos] = LL_USART_ReceiveData8(USARTx);
//		  }
//		  else
//		  {
//			  rx_dt[pos - 1] = LL_USART_ReceiveData8(USARTx);
//		  }
//		  pos++;
//	  }
//	  snprintf(dataForTransmitt, sizeof(dataForTransmitt), "%c\n", rx_dt[1]);
//	  transmitData(UART5, (uint8_t*) dataForTransmitt);
//	  return pos;
//}

void speedControl(void){

//	float coeff = 0.00;
	static float speedControlled = 0;

	int32_t error = (speedCart_target - speedCart);
	speedControlled += speed_coeff * error;

	speedControlled = constrain_sign(speedControlled, 0, 500);

	moveCarridge((int32_t)speedControlled);

}



void speedController(void)
{
	float kp = 0.1;
	float ki = 0.000;
	float kd = 0.1;

	float speedErr = 0;
	float diff = 0;
	int32_t neededSpeed = 0;

	if (speedPid != 0)
	{
		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_1); // Enable moving
		speedErr = ((float)speedPid) - previousSpeed;
		previousSpeed = getCartSpeed();
		diff = speedErr - prevSpeedErr;
		prevSpeedErr = speedErr;

		integral = (integral + speedErr);

		if ((int32_t)integral > 100)
			integral = 100;
		else if ((int32_t)integral < -100)
			integral = 100;

		neededSpeed = (int32_t) ((speedErr * kp + diff * kd + integral* ki) + 0.5);

		if (neededSpeed < 0)
		{
			if (neededSpeed < -maxspeed)
				neededSpeed = -maxspeed;

			LL_TIM_OC_SetCompareCH3(TIM2, 0);            		       // Left moving
			LL_TIM_OC_SetCompareCH2(TIM2, (-((int32_t)neededSpeed)));  // Left moving
		}

		else if (neededSpeed > 0)
		{
			if (neededSpeed > maxspeed)
				neededSpeed = maxspeed;

			LL_TIM_OC_SetCompareCH2(TIM2, 0);            		       // Right moving
			LL_TIM_OC_SetCompareCH3(TIM2, (uint32_t)(neededSpeed));    // Right moving
		}

		else if (neededSpeed == 0)
		{
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1); 			   // Disable moving
			LL_TIM_OC_SetCompareCH3(TIM2, 0);             			   // Stop moving
			LL_TIM_OC_SetCompareCH2(TIM2, 0);            			   // Stop moving
			speedPid = 0;
			previousSpeed = 0;
			prevSpeedErr = 0;
			integral = 0 ;
		}

	}
	else
	{
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1); 		// Disable moving
		LL_TIM_OC_SetCompareCH3(TIM2, 0);             		// Stop moving
		LL_TIM_OC_SetCompareCH2(TIM2, 0);            		// Stop moving
	}

//  	snprintf(dataForTransmitt, sizeof(dataForTransmitt), "%f\n", speedErr);
//  	transmitData(UART5, (uint8_t*) dataForTransmitt);
}

void moveCarridge(int32_t speed)
	{

	/*
	 * Function for speed control of DC motor
	 * moveCarridge(0); // Stop moving
	 * moveCarridge(-number); // Left moving
	 * moveCarridge(+number); // Right moving
	*/


	  int shift = 0;
	  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_1); // Enable moving

	  if (speed < 0)
	  {speed -= shift;}

	  if (speed == 0)
	  {
	      LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1); // Disable moving
	      LL_TIM_OC_SetCompareCH3(TIM2, 0);             // Stop moving
	      LL_TIM_OC_SetCompareCH2(TIM2, 0);             // Stop moving
	  }

	  else if (speed < 0)
	  {
	    if (speed < -maxspeed)
	    {speed = -maxspeed - shift;}
	    if (speed > -minspeed)
	    {speed = -minspeed - shift;}

	    LL_TIM_OC_SetCompareCH3(TIM2, 0);            			  // Left moving
	    LL_TIM_OC_SetCompareCH2(TIM2, (-((int32_t)speed)));       // Left moving
	  }
	  else if (speed > 0)
	  {
	    if (speed > maxspeed)
	    		speed = maxspeed;
	    if (speed < minspeed)
	    {speed = minspeed;}

	    LL_TIM_OC_SetCompareCH2(TIM2, 0);            			 // Right moving
	    LL_TIM_OC_SetCompareCH3(TIM2, (uint32_t)(speed));        // Right moving
	  }

	}

uint8_t moveToPosition(float setpoint)
{
	float kp = 0.01;
	float kd = 0.02;
	int32_t needed_speed = 0;

	float err = 0;
	float prevErr = 0;
	float diff = 0;


	err = setpoint - getTicksCartPosition();
	while(abs((int)err) > 100)
	{
		err = setpoint - getTicksCartPosition();
		integral += err;
		diff = (err - prevErr);
		prevErr = err;
		needed_speed = (int32_t) err * kp + diff * kd;

		if ((needed_speed == 0) && (err>0))
		{needed_speed = 1;}
		else if ((needed_speed == 0) & (err<0))
		{needed_speed = -1;}

		if (needed_speed < 0){
			needed_speed = constrain(needed_speed, -maxspeed, -110);
		} else {
			needed_speed = constrain(needed_speed, 110, maxspeed);
		}

		moveCarridge(needed_speed);
	}
	moveCarridge(0);
//	LL_mDelay(10);
	return 1;
}

int8_t poleCalibration(void)
{
	uint8_t tempCounter = 0;
	for(uint16_t i=0; i<450; i=i+2)
	{
		uint16_t firstPos = TIM4->CNT;
		LL_mDelay(50 + i);
		if (firstPos == TIM4->CNT)
			tempCounter++;

		if (tempCounter == 30)
		{
			TIM4->CNT = 32000;
			return 1;
		}
	}
	return -1;
}

uint8_t calibration(void)
{


	TIM5->CNT = 0;
	moveCarridge(0);
	__enable_irq ();
	isCalibration = 1;
	int32_t speed = 120;

	moveCarridge(-speed);
	uint16_t currPosition = TIM8->CNT;
	while(leftSwitch == 0)
	{
		if (TIM8->CNT - currPosition != 0)
			TIM5->CNT = 0;
		else
			if (TIM5->CNT > 100000 && !LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_9))
			{
				moveCarridge(speed);
				LL_mDelay(100);
				moveCarridge(0);
			}
		currPosition = TIM8->CNT;
	}
	__disable_irq ();
	moveCarridge(0);
	leftSwitch = 0;
	TIM8->CNT = 0;
	moveCarridge(speed);
	LL_mDelay(100);
	__enable_irq ();
	currPosition = TIM8->CNT;
	TIM5->CNT = 0;
	while(rightSwitch == 0){
		if (TIM8->CNT - currPosition != 0)
			TIM5->CNT = 0;
		else
			if (TIM5->CNT > 100000 && !LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_15))
			{
				moveCarridge(-speed);
				LL_mDelay(100);
				moveCarridge(0);
			}
		currPosition = TIM8->CNT;
	}
	__disable_irq ();
	moveCarridge(0);
	rightSwitch = 0;
	cartMovLength = (float)TIM8->CNT;


	if (cartMovLength < 22000)
	{
		moveCarridge(0);
		isCalibration = 0;
		LL_mDelay (2000);
		cartMovLength = 0;
		repeatCalibrationFlag++;
		calibration();
	}
	else
	{
		TIM8->CNT += SHIFT;
		moveToPosition(cartMovLength/2);
		LL_mDelay(50);
		__enable_irq ();
		TIM8->CNT = SHIFT;
		isCalibration = 0;
	}

	if (repeatCalibrationFlag==0)
		{
		while (poleCalibration() != 1)
		;
		return (1);
		}
	else
		repeatCalibrationFlag--;

	moveCarridge(0);


}

int16_t getTicksCartPosition(void)
{
	return (TIM8->CNT - SHIFT);
}

int16_t getTicksPolePosition(void)
{
	return (TIM4->CNT - SHIFT);
}

float getCartSpeed(void)
{
	/*
	 * Send to computer current speed
	 * Speed shows as ticks per microseconds
	*/

	return ((18.8495559 * (float)speedCart) / timeStep); // Meters per second
//	return ((float)time);
}

float getPoleSpeed(void)
{

	return ((1533.98079 * (float)speedPole) / timeStep); // Ticks per second
//	return ((float)time);
}

void start_melody(void)
{
	uint8_t delay_ms = 16;

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	LL_mDelay(70);

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	GPIOE->ODR |= (1<<9);
	LL_mDelay(delay_ms);
	GPIOE->ODR &= ~(1<<9);
	LL_mDelay(delay_ms);

	LL_mDelay(200);
}

//long double getMetersCartPosition(void)
//{
//	/*
//	 * Pulley Diameter = 0,012 meters
//	 * Length of 1 circle = Pi * Diameter (0,012 * 3.1415927)
//	 * Ticks per round = 2000 ticks of encoder
//	 * Length of circle in meters = 0.037699111843078
//	 * TICK_METERS_LENGTH = circleLength / Ticks per round = 0.000018849555922;
//	*/
//	return ((long double)(TIM8->CNT - SHIFT)) * 0.000018849555922;
//}

//long double getRadianPolePosition(void)
//{
//	/*
//	 * 1 round = 4096 ticks of encoder
//	 * 360 degree / 4096 = 0,087890625 degree per 1 encoder`s tick
//	 * 1 tick in radians = 0.00153398079 rad
//	 */
//	return (((long double)(TIM4->CNT - SHIFT)) * 0.0015707963267948964);
//}
