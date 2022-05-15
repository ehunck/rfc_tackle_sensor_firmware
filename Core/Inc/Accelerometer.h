/*
 * Accelerometer.h
 *
 *  Created on: May 14, 2022
 *      Author: eddiehunckler
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} Accelerometer_RawData;

typedef struct
{
	float x;
	float y;
	float z;
} Accelerometer_Data;

bool Accelerometer_Init();
void Accelerometer_Update();

bool Accelerometer_GetData( Accelerometer_Data *data );
bool Accelerometer_GetRawData( Accelerometer_RawData *data );

float Accelerometer_GetMagnitude();

#endif /* INC_ACCELEROMETER_H_ */
