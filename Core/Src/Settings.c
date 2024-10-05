/*
 * Settings.c
 *
 *  Created on: Aug 16, 2023
 *      Author: eddiehunckler
 */

#include "Settings.h"
#include "ee.h"

#define VIRTAUL_ADDR_RGB	0
#define DATA_SET_MAGIC		0xAB

typedef struct __attribute__((__packed__))
{
	uint8_t set;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t fade;
} RGBData;

static RGBData _data;

void Settings_Init()
{
	ee_init();
	if( ee_read(VIRTAUL_ADDR_RGB, sizeof(_data), (uint8_t*)&_data ) )
	{
		if( _data.set != DATA_SET_MAGIC )
		{
			// set defaults
			_data.red = 0;
			_data.green = 255;
			_data.blue = 0;
			_data.fade = 0;
			_data.set = DATA_SET_MAGIC;
			ee_format(false);
			ee_write(VIRTAUL_ADDR_RGB, sizeof(_data), (uint8_t*)&_data );
		}
	}
}

void Settings_SetHomeRedGreenBlue(uint8_t r, uint8_t g, uint8_t b)
{
	bool save_needed = false;
	if( _data.red != r )
	{
		_data.red = r;
		save_needed = true;
	}
	if( _data.green != g )
	{
		_data.green = g;
		save_needed = true;
	}
	if( _data.blue != b )
	{
		_data.blue = b;
		save_needed = true;
	}
	if( save_needed )
	{
		_data.set = DATA_SET_MAGIC;
		ee_format(false);
		ee_write(VIRTAUL_ADDR_RGB, sizeof(_data), (uint8_t*)&_data );
	}
}

void Settings_SetFade(uint8_t fade_enable)
{
	bool save_needed = false;
	if( _data.fade != fade_enable )
	{
		_data.fade = fade_enable;
		save_needed = true;
	}
	if( save_needed )
	{
		_data.set = DATA_SET_MAGIC;
		ee_format(false);
		ee_write(VIRTAUL_ADDR_RGB, sizeof(_data), (uint8_t*)&_data );
	}
}

uint8_t Settings_GetHomeRed()
{
	return _data.red;
}

uint8_t Settings_GetHomeGreen()
{
	return _data.green;
}

uint8_t Settings_GetHomeBlue()
{
	return _data.blue;
}

uint8_t Settings_GetFade()
{
	return _data.fade;
}

