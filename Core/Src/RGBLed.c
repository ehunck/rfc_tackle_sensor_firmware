/*
 * RGBLed.c
 *
 *  Created on: May 13, 2022
 *      Author: eddiehunckler
 */

#include "RGBLed.h"
#include "main.h"

#define RED_CHANNEL		TIM_CHANNEL_2
#define GREEN_CHANNEL	TIM_CHANNEL_4
#define BLUE_CHANNEL	TIM_CHANNEL_1

extern TIM_HandleTypeDef htim1;

uint16_t ScaleRGBToPulse(uint8_t val)
{
	// Assumes the value coming in is 0 for 0% or 255 for 100%
	// Pulse spans 0 for 0% and 1000 for 100%
	uint32_t temp = val*1000;
	return (uint16_t) (temp/255);
}

void RGBLed_Init()
{
	// Todo: Currently assume that the main function
	// is initializing the peripherals
	RGBLed_SetOff();
}

void RGBLed_SetManual(uint8_t r, uint8_t g, uint8_t b)
{
	if( r )
	{
		__HAL_TIM_SET_COMPARE(&htim1, RED_CHANNEL, ScaleRGBToPulse(r));
		HAL_TIM_PWM_Start(&htim1, RED_CHANNEL);
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim1, RED_CHANNEL);
	}
	if( g )
	{
		__HAL_TIM_SET_COMPARE(&htim1, GREEN_CHANNEL, ScaleRGBToPulse(g));
		HAL_TIM_PWM_Start(&htim1, GREEN_CHANNEL);
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim1, GREEN_CHANNEL);
	}
	if( b )
	{
		__HAL_TIM_SET_COMPARE(&htim1, BLUE_CHANNEL, ScaleRGBToPulse(b));
		HAL_TIM_PWM_Start(&htim1, BLUE_CHANNEL);
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim1, BLUE_CHANNEL);
	}
}

void RGBLed_SetRed()
{
	RGBLed_SetManual(255, 0, 0);
}

void RGBLed_SetGreen()
{
	RGBLed_SetManual(0, 255, 0);
}

void RGBLed_SetBlue()
{
	RGBLed_SetManual(0, 0, 255);
}

void RGBLed_SetYellow()
{
	RGBLed_SetManual(255, 255, 0);
}

void RGBLed_SetCyan()
{
	RGBLed_SetManual(0, 255, 255);
}

void RGBLed_SetMagenta()
{
	RGBLed_SetManual(255, 0, 255);
}

void RGBLed_SetWhite()
{
	RGBLed_SetManual(255, 255, 255);
}

void RGBLed_SetOff()
{
	HAL_TIM_PWM_Stop(&htim1, RED_CHANNEL);
	HAL_TIM_PWM_Stop(&htim1, GREEN_CHANNEL);
	HAL_TIM_PWM_Stop(&htim1, BLUE_CHANNEL);
}
