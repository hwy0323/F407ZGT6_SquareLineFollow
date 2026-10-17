#ifndef MENU_COMMAND_H
#define MENU_COMMAND_H
#include <stdint.h>
typedef struct {char line[24];uint8_t used,collecting;} MenuCommand;
/* 1=启动当前项，2=下一项。单字节 G/T 或缺换行不启动。 */
uint8_t menu_command_feed(MenuCommand *parser,uint8_t byte);
#endif
