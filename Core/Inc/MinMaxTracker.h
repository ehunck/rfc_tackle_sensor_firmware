/*
 * MinMaxTracker.h
 *
 *  Created on: Oct 22, 2023
 *      Author: eddiehunckler
 */

#ifndef INC_MINMAXTRACKER_H_
#define INC_MINMAXTRACKER_H_

#include <stdio.h>
#include <stdbool.h>
#include  <float.h>

typedef struct {
	float min;
	float max;
	bool isInitialized;
} MinMaxTracker;

void MinMaxTracker_Reset(MinMaxTracker* ctx);
void MinMaxTracker_Update(MinMaxTracker* ctx, float value);
float MinMaxTracker_getMin(MinMaxTracker *ctx);
float MinMaxTracker_getMax(MinMaxTracker *ctx);

#endif /* INC_MINMAXTRACKER_H_ */
