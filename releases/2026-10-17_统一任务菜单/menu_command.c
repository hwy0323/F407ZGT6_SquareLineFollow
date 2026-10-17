#include "menu_command.h"
#include <string.h>
uint8_t menu_command_feed(MenuCommand *p,uint8_t byte)
{
  if(byte=='@'){p->used=0;p->collecting=1;}
  if(!p->collecting || byte=='\r')return 0;
  if(byte=='\n'){
    p->line[p->used]=0;p->collecting=0;
    return !strcmp(p->line,"@MENU:RUN")?1:!strcmp(p->line,"@MENU:NEXT")?2:0;
  }
  if(byte<32 || byte>126 || p->used>=sizeof(p->line)-1U){p->collecting=p->used=0;return 0;}
  p->line[p->used++]=(char)byte;return 0;
}
