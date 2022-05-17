/*
 * UserTimer.h
 *
 *  Created on: May 16, 2022
 *      Author: eddiehunckler
 */

#ifndef INC_USERTIMER_H_
#define INC_USERTIMER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	uint32_t start_time_ms;
	uint32_t interval_ms;
	bool enabled;
} UserTimer;

void UserTimer_Init(UserTimer* ctx, uint32_t interval_ms);
void UserTimer_Start(UserTimer* ctx);
void UserTimer_Stop(UserTimer* ctx);
bool UserTimer_GetActive(UserTimer* ctx);

#endif /* INC_USERTIMER_H_ */
