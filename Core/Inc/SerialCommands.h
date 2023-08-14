/*
 * SerialCommands.h
 *
 *  Created on: Jul 21, 2023
 *      Author: eddiehunckler
 */

#ifndef INC_SERIALCOMMANDS_H_
#define INC_SERIALCOMMANDS_H_

#include <stdint.h>
#include <stdbool.h>

#define BUFFER_SIZE 100

typedef struct 
{
    const char* command_str;
    void (*command_func)(const char* msg, uint32_t msg_len);
} Command;

typedef struct
{
    const Command* commands;
    uint8_t num_commands;
    const char* delimiter;
    char buffer[BUFFER_SIZE];
    uint8_t buffer_index;
} SerialCommands;

bool SerialCommands_Init(SerialCommands* ctx, const Command* commands, uint8_t num_commands, const char* delimiter );
bool SerialCommands_ReceiveMessage(SerialCommands* ctx, const char* msg, uint8_t msg_len);
bool SerialCommands_Process(SerialCommands* ctx);

#endif /* INC_SERIALCOMMANDS_H_ */
