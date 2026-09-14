#ifndef MENU_DISPLAY_H
#define MENU_DISPLAY_H

#include "menu.h"

void MenuDisplay_Init(void);
void MenuDisplay_Show_Browse(MenuTaskId selected_task);
void MenuDisplay_Show_Running(MenuTaskId task);
void MenuDisplay_Show_Stopped(MenuTaskId task);

#endif /* MENU_DISPLAY_H */
