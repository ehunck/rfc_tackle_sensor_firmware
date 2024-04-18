/*
 * RGBLed.c
 *
 *  Created on: May 13, 2022
 *      Author: eddiehunckler
 */

#include "RGBLed.h"
#include "main.h"
#include <stdbool.h>

#define RED_CHANNEL		TIM_CHANNEL_2
#define GREEN_CHANNEL	TIM_CHANNEL_4
#define BLUE_CHANNEL	TIM_CHANNEL_1

extern TIM_HandleTypeDef htim1;

bool pulse_mode_enabled = false;
uint8_t red, green, blue = 0;

uint16_t ScaleRGBToPulse(uint8_t val)
{
	// Assumes the value coming in is 0 for 0% or 255 for 100%
	// Pulse spans 0 for 0% and 1000 for 100%
	uint32_t temp = val*1000;
	return (uint16_t) (temp/255);
}

void RGBLed_Init()
{
	// Currently assume that the main function
	// is initializing the peripherals
	RGBLed_SetOff();
}

void RGBLed_SetManual(uint8_t r, uint8_t g, uint8_t b, bool refresh)
{
	red = r;
	green = g;
	blue = b;
	if( refresh )
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
}

void RGBLed_SetRed(bool refresh)
{
	RGBLed_SetManual(255, 0, 0, refresh);
}

void RGBLed_SetGreen(bool refresh)
{
	RGBLed_SetManual(0, 255, 0, refresh);
}

void RGBLed_SetBlue(bool refresh)
{
	RGBLed_SetManual(0, 0, 255, refresh);
}

void RGBLed_SetWhite(bool refresh)
{
	RGBLed_SetManual(255, 255, 255, refresh);
}

void RGBLed_SetOff()
{
	HAL_TIM_PWM_Stop(&htim1, RED_CHANNEL);
	HAL_TIM_PWM_Stop(&htim1, GREEN_CHANNEL);
	HAL_TIM_PWM_Stop(&htim1, BLUE_CHANNEL);
}

void RGBLed_EnablePulse()
{
	pulse_mode_enabled = true;
}

void RGBLed_DisablePulse()
{
	pulse_mode_enabled = false;
}

const int max_steps = 50;
int step = 0;
const uint16_t sineLookupTable[] = {
25, 28, 31, 34, 37, 40, 42, 44,
46, 48, 49, 50, 50, 50, 50, 49,
48, 46, 44, 42, 40, 37, 34, 31,
28, 25, 22, 19, 16, 13, 10, 8,
6, 4, 2, 1, 0, 0, 0, 0,
1, 2, 4, 6, 8, 10, 13, 16,
19, 22};

void RGBLed_IRQUpdate()
{
	if( pulse_mode_enabled )
	{
		RGBLed_SetManual(
				(red*sineLookupTable[step])/(4*max_steps),
				(green*sineLookupTable[step])/(4*max_steps),
				(blue*sineLookupTable[step])/(4*max_steps),
				true);

		step++;
		if( step >= max_steps )
		{
			step = 0;
		}
	}
}
