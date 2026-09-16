#ifndef MENU_H
#define MENU_H

#include <stdint.h>

/*
 * Six-task menu core.
 *
 * This file deliberately does not depend on an OLED driver. When an OLED is
 * added later, its display code can call Menu_GetSelectedTask() and
 * Menu_GetSelectedLabel() directly.
 */
typedef enum {
  MENU_TASK_1_SQUARE_LINE = 1U,
  MENU_TASK_2_SIDE_PARKING,
  MENU_TASK_3_REVERSE_PARKING,
  MENU_TASK_4_ENCODER_TEST,
  MENU_TASK_5_RESERVED,
  MENU_TASK_6_RESERVED
} MenuTaskId;

void Menu_Init(void);
void Menu_Select_First(void);
void Menu_Select_Next(void);
MenuTaskId Menu_GetSelectedTask(void);
const char *Menu_GetSelectedLabel(void);
uint8_t Menu_Task_Is_Ready(MenuTaskId task);

#endif /* MENU_H */
