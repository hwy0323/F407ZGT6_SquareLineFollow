#include "start_command.h"
#include <string.h>
uint8_t start_command_feed(StartCommand *command, uint8_t byte)
{
  if(byte=='@'){command->used=0U;command->collecting=1U;}
  if(!command->collecting || byte=='\r')return 0U;
  if(byte=='\n'){
    command->line[command->used]=0;
    command->collecting=0U;
    return strcmp(command->line,TASK4_START_FRAME)==0;
  }
  if(byte<32U || byte>126U || command->used>=sizeof(command->line)-1U){
    command->used=command->collecting=0U;return 0U;
  }
  command->line[command->used++]=(char)byte;
  return 0U;
}
