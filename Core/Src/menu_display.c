#include "menu_display.h"
#include "oled.h"

static const char * const menu_lines[] = {
  "SQUARE LINE",
  "SIDE PARK DEMO",
  "RESERVED 3",
  "RESERVED 4",
  "RESERVED 5",
  "RESERVED 6"
};

static void Draw_Task_Line(uint8_t row, MenuTaskId task, uint8_t selected)
{
  OLED_Draw_String(row, 0U, selected ? ">" : " ");
  OLED_Draw_String(row, 2U, menu_lines[(uint32_t)task - 1U]);
}

void MenuDisplay_Init(void)
{
  (void)OLED_Init();
}

void MenuDisplay_Show_Browse(MenuTaskId selected_task)
{
  uint8_t row;

  if (!OLED_Is_Ready()) return;
  OLED_Clear();
  OLED_Draw_String(0U, 0U, "MENU  TASK");
  OLED_Draw_String(0U, 11U, Menu_GetSelectedLabel());
  for (row = 0U; row < 6U; row++) {
    MenuTaskId task = (MenuTaskId)(row + 1U);
    Draw_Task_Line((uint8_t)(row + 1U), task, task == selected_task);
  }
  OLED_Draw_String(7U, 0U, "HOLD 3S GO");
  OLED_Refresh();
}

void MenuDisplay_Show_Running(MenuTaskId task)
{
  if (!OLED_Is_Ready()) return;
  OLED_Clear();
  OLED_Draw_String(1U, 0U, "RUNNING TASK");
  OLED_Draw_String(1U, 14U, Menu_GetSelectedLabel());
  OLED_Draw_String(3U, 0U, menu_lines[(uint32_t)task - 1U]);
  OLED_Draw_String(6U, 0U, "PRESS KEY STOP");
  OLED_Refresh();
}

void MenuDisplay_Show_Stopped(MenuTaskId task)
{
  MenuDisplay_Show_Browse(task);
}
