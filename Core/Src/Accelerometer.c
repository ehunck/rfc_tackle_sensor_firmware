/*
 * Accelerometer.c
 *
 *  Created on: May 14, 2022
 *      Author: eddiehunckler
 */

#include "Accelerometer.h"

//typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
//typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
//
//typedef struct
//{
//  /** Component mandatory fields **/
//  stmdev_write_ptr  write_reg;
//  stmdev_read_ptr   read_reg;
//  /** Customizable optional pointer **/
//  void *handle;
//} stmdev_ctx_t;

Accelerometer_Data _latest_data;

void Accelerometer_Init()
{

}

void Accelerometer_Update()
{

}

bool Accelerometer_GetData( Accelerometer_Data *data )
{
	return false;
}
