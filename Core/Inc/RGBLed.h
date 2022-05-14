/*
 * RGBLed.h
 *
 *  Created on: May 13, 2022
 *      Author: eddiehunckler
 */

#ifndef INC_RGBLED_H_
#define INC_RGBLED_H_

#include <stdint.h>

void RGBLed_Init();
void RGBLed_SetManual(uint8_t r, uint8_t g, uint8_t b);

void RGBLed_SetRed();
void RGBLed_SetGreen();
void RGBLed_SetBlue();
void RGBLed_SetYellow();
void RGBLed_SetCyan();
void RGBLed_SetMagenta();
void RGBLed_SetWhite();
void RGBLed_SetOff();

#endif /* INC_RGBLED_H_ */
