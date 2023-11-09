/*
 * Accelerometer.c
 *
 *  Created on: May 14, 2022
 *      Author: eddiehunckler
 */


#include "main.h"
#include <string.h>
#include <stdio.h>
#include "Accelerometer.h"
#include "lis2de12_reg.h"
#include <math.h>

extern SPI_HandleTypeDef hspi1;

int32_t SPIWriteInterface(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
	reg |= 0x40;
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(handle, (uint8_t*) buf, len, HAL_MAX_DELAY);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	return 0;
}

int32_t SPIReadInterface(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
	reg |= 0xC0;
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(handle, buf, len, HAL_MAX_DELAY);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	return 0;
}

stmdev_ctx_t spi_interface = {
		SPIWriteInterface,
		SPIReadInterface,
		(void*)&hspi1
};

static int16_t data_raw_acceleration[3] = {0};
static int16_t data_raw_temperature = 0;
static float acceleration_mg[3] = {0};
static float temperature_degC = 0;
static uint8_t whoamI = 0;

bool Accelerometer_Init()
{
	// Assume that the SPI peripheral is setup in the main application.

	/* Check device ID */
	lis2de12_device_id_get(&spi_interface, &whoamI);

	if (whoamI != LIS2DE12_ID)
	{
		return false;
	}

	/* Enable Block Data Update */
	lis2de12_block_data_update_set(&spi_interface, PROPERTY_ENABLE);
	/* Set Output Data Rate to 100Hz */
	lis2de12_data_rate_set(&spi_interface, LIS2DE12_ODR_100Hz);
	/* Set full scale to 2g */
	lis2de12_full_scale_set(&spi_interface, LIS2DE12_8g);

	return true;
}

bool Accelerometer_Update()
{
	bool updated = false;
	lis2de12_reg_t reg;
	/* Read output only if new value available */
	lis2de12_xl_data_ready_get(&spi_interface, &reg.byte);

	if (reg.byte)
	{
		/* Read accelerometer data */
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lis2de12_acceleration_raw_get(&spi_interface, data_raw_acceleration);
		acceleration_mg[0] =
				lis2de12_from_fs8_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] =
				lis2de12_from_fs8_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] =
				lis2de12_from_fs8_to_mg(data_raw_acceleration[2]);

		updated = true;
	}
	return updated;
}

bool Accelerometer_GetData( Accelerometer_Data *data )
{
	if( data )
	{
		data->x = acceleration_mg[0];
		data->y = acceleration_mg[1];
		data->z = acceleration_mg[2];

		return true;
	}
	return false;
}

bool Accelerometer_GetRawData( Accelerometer_RawData *data )
{
	if( data )
	{
		data->x = data_raw_acceleration[0];
		data->y = data_raw_acceleration[1];
		data->z = data_raw_acceleration[2];

		return true;
	}
	return false;
}

float Accelerometer_GetMagnitude()
{
	float sq = acceleration_mg[0]*acceleration_mg[0] + acceleration_mg[1]*acceleration_mg[1] + acceleration_mg[2]*acceleration_mg[2];
	float mag = sqrtf(sq);
	return mag;
}
