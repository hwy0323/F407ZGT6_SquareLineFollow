#ifndef TASK4_START_COMMAND_H
#define TASK4_START_COMMAND_H
#include <stdint.h>
#define TASK4_START_FRAME "@TASK4:VISION:RUN"
typedef struct { char line[32]; uint8_t used, collecting; } StartCommand;
uint8_t start_command_feed(StartCommand *command, uint8_t byte);
#endif
