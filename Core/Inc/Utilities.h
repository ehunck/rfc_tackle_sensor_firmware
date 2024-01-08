/*
 * Utilities.h
 *
 *  Created on: Jan 4, 2024
 *      Author: eddiehunckler
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

int Clamp( int val, int min, int max );
float IIRFilter( float x, float y_prev, float alpha );

#endif /* INC_UTILITIES_H_ */
