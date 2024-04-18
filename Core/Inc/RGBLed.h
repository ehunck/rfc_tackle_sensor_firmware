/*
 * RGBLed.h
 *
 *  Created on: May 13, 2022
 *      Author: eddiehunckler
 */

#ifndef INC_RGBLED_H_
#define INC_RGBLED_H_

#include <stdint.h>
#include <stdbool.h>

void RGBLed_Init();
void RGBLed_SetManual(uint8_t r, uint8_t g, uint8_t b, bool refresh);

void RGBLed_SetRed(bool refresh);
void RGBLed_SetGreen(bool refresh);
void RGBLed_SetBlue(bool refresh);
void RGBLed_SetWhite(bool refresh);

void RGBLed_SetOff();

void RGBLed_EnablePulse();
void RGBLed_DisablePulse();

void RGBLed_IRQUpdate();

#endif /* INC_RGBLED_H_ */
