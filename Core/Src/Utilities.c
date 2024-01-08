/*
 * Utilities.c
 *
 *  Created on: Jan 4, 2024
 *      Author: eddiehunckler
 */

#include "Utilities.h"

#include <stdint.h>
#include <stdio.h>

// Scale factor for Q31 format
#define Q31_SCALE_FACTOR (2147483648LL) // 2^31

// Convert float to Q31 format
int32_t float_to_q31(float number)
{
    return (int32_t)(number * Q31_SCALE_FACTOR);
}

// Convert Q31 format to float
float q31_to_float(int32_t number)
{
    return (float)number / Q31_SCALE_FACTOR;
}

// Core IIR Filter function in fixed-point arithmetic
int32_t IIRFilterFixed(int32_t x, int32_t y_prev, int32_t alpha)
{
    // Perform fixed-point multiplication and addition
    int64_t result = (int64_t)alpha * x + (int64_t)(Q31_SCALE_FACTOR - alpha) * y_prev;

    // Scale back to Q31 format and round the result
    result = (result + (Q31_SCALE_FACTOR >> 1)) >> 31; // Add half for rounding

    return (int32_t)result;
}

// Wrapper IIR Filter function with floating-point interface
float IIRFilter(float x, float y_prev, float alpha)
{
    // Convert float to Q31
    int32_t x_fixed = float_to_q31(x);
    int32_t y_prev_fixed = float_to_q31(y_prev);
    int32_t alpha_fixed = float_to_q31(alpha);

    // Call the fixed-point IIR filter function
    int32_t y_fixed = IIRFilterFixed(x_fixed, y_prev_fixed, alpha_fixed);

    // Convert Q31 to float and return
    return q31_to_float(y_fixed);
}

int Clamp( int val, int min, int max )
{
	if( val > max )
	{
		return max;
	}
	if( val < min )
	{
		return min;
	}
	return val;
}

