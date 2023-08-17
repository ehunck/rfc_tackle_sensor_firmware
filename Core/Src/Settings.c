/*
 * Settings.c
 *
 *  Created on: Aug 16, 2023
 *      Author: eddiehunckler
 */

#include "Settings.h"

static uint8_t _red;
static uint8_t _green;
static uint8_t _blue;

void Settings_Init()
{
	_red = 0;
	_green = 255;
	_blue = 0;
}

void Settings_SetHomeRedGreenBlue(uint8_t r, uint8_t g, uint8_t b)
{
	_red = r;
	_green = g;
	_blue = b;
}

uint8_t Settings_GetHomeRed()
{
	return _red;
}

uint8_t Settings_GetHomeGreen()
{
	return _green;
}

uint8_t Settings_GetHomeBlue()
{
	return _blue;
}
