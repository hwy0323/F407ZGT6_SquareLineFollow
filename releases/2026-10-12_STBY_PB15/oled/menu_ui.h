#ifndef MENU_UI_H
#define MENU_UI_H
#include <stdint.h>

typedef enum { MENU_VIEW = 0U, MENU_SCROLL = 1U, MENU_PREVIEW = 2U } MenuView;
#define MENU_ITEM_COUNT 11U
#define MENU_SCROLL_MS 240U
#define MENU_PREVIEW_MS 2000U

typedef struct {
  uint8_t selected, scroll_from, queued_steps, dirty;
  MenuView view;
  uint32_t view_start_ms, preview_count, completion_count;
} MenuUi;

void MenuUi_Init(MenuUi *ui);
void MenuUi_Next(MenuUi *ui, uint32_t now);
void MenuUi_Confirm(MenuUi *ui, uint32_t now);
void MenuUi_Tick(MenuUi *ui, uint32_t now);
void MenuUi_Draw(const MenuUi *ui, uint32_t now);
const char *MenuUi_Label(uint8_t index);
#endif
