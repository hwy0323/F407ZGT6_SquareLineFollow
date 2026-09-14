#include "menu.h"

static MenuTaskId selected_task = MENU_TASK_1_SQUARE_LINE;

static const char * const task_labels[] = {
  "Task 1: square line follow",
  "Task 2: side parking demo",
  "Task 3: reserved",
  "Task 4: reserved",
  "Task 5: reserved",
  "Task 6: reserved"
};

void Menu_Init(void)
{
  selected_task = MENU_TASK_1_SQUARE_LINE;
}

void Menu_Select_Next(void)
{
  if (selected_task >= MENU_TASK_6_RESERVED) {
    selected_task = MENU_TASK_1_SQUARE_LINE;
  } else {
    selected_task = (MenuTaskId)((uint32_t)selected_task + 1U);
  }
}

MenuTaskId Menu_GetSelectedTask(void)
{
  return selected_task;
}

const char *Menu_GetSelectedLabel(void)
{
  return task_labels[(uint32_t)selected_task - 1U];
}

uint8_t Menu_Task_Is_Ready(MenuTaskId task)
{
  return (task == MENU_TASK_1_SQUARE_LINE ||
          task == MENU_TASK_2_SIDE_PARKING) ? 1U : 0U;
}
