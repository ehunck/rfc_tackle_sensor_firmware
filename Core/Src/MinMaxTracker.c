/*
 * MinMaxTracker.c
 *
 *  Created on: Oct 22, 2023
 *      Author: eddiehunckler
 */

#include "MinMaxTracker.h"


void MinMaxTracker_Reset(MinMaxTracker* ctx)
{
	ctx->min = FLT_MAX;
    ctx->max = -FLT_MAX;
    ctx->isInitialized = false;
}

// Updates the tracker with a new floating point value
void MinMaxTracker_Update(MinMaxTracker* ctx, float value)
{
    if (!ctx->isInitialized)
    {
    	ctx->min = value;
    	ctx->max = value;
    	ctx->isInitialized = true;
    }
    else
    {
        if (value < ctx->min)
        {
        	ctx->min = value;
        }
        if (value > ctx->max)
        {
        	ctx->max = value;
        }
    }
}

// Retrieves the minimum value
float MinMaxTracker_getMin(MinMaxTracker *ctx)
{
    if (!ctx->isInitialized)
    {
        return -1;  // Indicate error. Depending on application, you might handle this differently.
    }
    return ctx->min;
}

// Retrieves the maximum value
float MinMaxTracker_getMax(MinMaxTracker *ctx)
{
    if (!ctx->isInitialized)
    {
        return -1;  // Indicate error. Depending on application, you might handle this differently.
    }
    return ctx->max;
}

