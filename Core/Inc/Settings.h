/*
 * Settings.h
 *
 *  Created on: Aug 16, 2023
 *      Author: eddiehunckler
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include <stdint.h>

void Settings_Init();
void Settings_SetHomeRedGreenBlue(uint8_t r, uint8_t g, uint8_t b);
uint8_t Settings_GetHomeRed();
uint8_t Settings_GetHomeGreen();
uint8_t Settings_GetHomeBlue();

#endif /* INC_SETTINGS_H_ */
