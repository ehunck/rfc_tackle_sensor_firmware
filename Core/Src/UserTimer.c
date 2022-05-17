/*
 * UserTimer.c
 *
 *  Created on: May 16, 2022
 *      Author: eddiehunckler
 */

#include "UserTimer.h"

void UserTimer_Init(UserTimer* ctx, uint32_t interval_ms)
{
	if(!ctx)
	{
		return;
	}
	ctx->interval_ms = interval_ms;
	ctx->start_time_ms = HAL_GetTick();
	ctx->enabled = false;
}

void UserTimer_Start(UserTimer* ctx)
{
	if(!ctx)
	{
		return;
	}
	ctx->start_time_ms = HAL_GetTick();
	ctx->enabled = true;
}

void UserTimer_Stop(UserTimer* ctx)
{
	if(!ctx)
	{
		return;
	}
	ctx->enabled = false;
}

bool UserTimer_GetActive(UserTimer* ctx)
{
	if(!ctx)
	{
		return false;
	}
	if(ctx->enabled
		&& ( HAL_GetTick() - ctx->start_time_ms ) <= ctx->interval_ms)
	{
		return true;
	}
	else
	{
		return false;
	}
}
