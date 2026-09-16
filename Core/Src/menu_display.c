#include "menu_display.h"
#include "oled.h"

static const char * const task_codes[] = {
  "T1", "T2", "T3", "T4", "T5", "T6"
};

static void Show_Full_Screen_Text(const char *text, uint8_t scale, uint8_t x)
{
  OLED_Clear();
  OLED_Draw_Big_String(x, 18U, scale, text);
  OLED_Refresh();
}

void MenuDisplay_Init(void)
{
  (void)OLED_Init();
}

void MenuDisplay_Show_Browse(MenuTaskId selected_task)
{
  if (!OLED_Is_Ready()) return;
  Show_Full_Screen_Text(task_codes[(uint32_t)selected_task - 1U], 6U, 28U);
}

void MenuDisplay_Show_Running(MenuTaskId task)
{
  (void)task;
  if (!OLED_Is_Ready()) return;
  Show_Full_Screen_Text("START", 4U, 4U);
}

void MenuDisplay_Show_Stopped(MenuTaskId task)
{
  MenuDisplay_Show_Browse(task);
}
