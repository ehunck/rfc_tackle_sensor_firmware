/*
 * SerialCommands.c
 *
 *  Created on: Jul 21, 2023
 *      Author: eddiehunckler
 */

#include "SerialCommands.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

int min(int a, int b)
{
    return a < b ? a : b;
}

int max(int a, int b)
{
    return a > b ? a : b;
}

void ResetBuffer(SerialCommands* ctx)
{
    ctx->buffer_index = 0;
    memset(ctx->buffer, 0, sizeof(ctx->buffer));
}

bool SerialCommands_Init(SerialCommands* ctx, const Command* commands, uint8_t num_commands, const char* delimiter )
{
    if(ctx == NULL || commands == NULL || num_commands == 0)
    {
        return false;
    }
    ctx->commands = commands;
    ctx->num_commands = num_commands;
    ctx->delimiter = delimiter;
    ResetBuffer(ctx);
    return true;
}

bool SerialCommands_ReceiveMessage(SerialCommands* ctx, const char* msg, uint8_t msg_len)
{
    if(ctx == NULL || msg == NULL)
    {
        return false;
    }

    // Append msg to buffer
    if( (ctx->buffer_index + msg_len) > BUFFER_SIZE)
    {
        ResetBuffer(ctx);
    }
    // Copy into buffer
    for( int i = 0; i < msg_len; i ++ )
    {
    	ctx->buffer[ctx->buffer_index + i] = msg[i];
    }
    ctx->buffer_index += msg_len;

    return true;
}

bool SerialCommands_Process(SerialCommands* ctx)
{
    if(ctx == NULL)
    {
        return false;
    }

    // Check if delimiter is in buffer
    char* delimiter_ptr = strstr(ctx->buffer, ctx->delimiter);
    if(delimiter_ptr == NULL)
    {
        return false;
    }
    while( delimiter_ptr )
    {
        // delimiter found
        // string length
        int str_len = delimiter_ptr - ctx->buffer + strlen(ctx->delimiter);

        // Check for commands
        bool commands_found = false;
        for(uint8_t i = 0; i < ctx->num_commands; i++)
        {
            int compare_len = min(strlen(ctx->commands[i].command_str), str_len);
            if(strncmp(ctx->buffer, ctx->commands[i].command_str, compare_len) == 0)
            {
                ctx->commands[i].command_func( ctx->buffer+compare_len, str_len-compare_len - strlen(ctx->delimiter) );
                // remove last command from the buffer
                memmove(ctx->buffer, ctx->buffer + str_len, BUFFER_SIZE - str_len);
                ctx->buffer_index -= str_len;
                // clear the rest of the buffer
                memset(ctx->buffer + ctx->buffer_index, 0, BUFFER_SIZE - ctx->buffer_index);
                commands_found = true;
                break;
            }
        }
        if( !commands_found )
		{
            // remove last command from the buffer
            memmove(ctx->buffer, ctx->buffer + str_len, BUFFER_SIZE - str_len);
            ctx->buffer_index -= str_len;
            // clear the rest of the buffer
            memset(ctx->buffer + ctx->buffer_index, 0, BUFFER_SIZE - ctx->buffer_index);
		}
        // Check for more delimiters
        delimiter_ptr = strstr(ctx->buffer, ctx->delimiter);
    }

    return true;
}
