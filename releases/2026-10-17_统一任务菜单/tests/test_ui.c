#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../menu_ui.h"
#include "../oled.h"
static unsigned draws,refreshes,rects;
static char text[3][12];
void OLED_Clear(void){draws=0;}
void OLED_Draw_Big_String(int16_t x,int16_t y,uint8_t scale,const char *s)
{
  (void)x;(void)y;(void)scale;assert(draws<3);
  snprintf(text[draws++],sizeof(text[0]),"%s",s);
}
void OLED_Draw_Rect(int16_t x,int16_t y,int16_t w,int16_t h)
{(void)x;(void)y;assert(w==114 && h==25);rects++;}
void OLED_Refresh(void){refreshes++;}
int main(void)
{
  MenuUi ui;MenuUi_Init(&ui);
  const char *labels[]={"1-1","1-2","2-1","2-2","3","4","5","6-1","6-2","6-3","6-4"};
  assert(MENU_ITEM_COUNT==11U);
  for(unsigned i=0;i<11;i++){
    ui.selected=i;MenuUi_Draw(&ui,0);
    assert(draws==3 && !strcmp(text[1],labels[i]));
    assert(!strcmp(text[0],labels[(i+10)%11]) && !strcmp(text[2],labels[(i+1)%11]));
  }
  assert(refreshes==11 && rects==11);
  puts("PASS OLED UI: all 11 task labels including 6-1/6-2/6-3/6-4, selected center and adjacent rows, preserved Git menu font/layout");
}
