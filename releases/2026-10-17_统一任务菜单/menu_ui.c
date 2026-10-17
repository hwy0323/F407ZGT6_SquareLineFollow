#include "menu_ui.h"
#include "oled.h"
#include <string.h>

static const char *const labels[MENU_ITEM_COUNT] = {
  "1-1", "1-2", "2-1", "2-2", "3", "4", "5", "6-1", "6-2", "6-3", "6-4"
};

const char *MenuUi_Label(uint8_t index) { return labels[index % MENU_ITEM_COUNT]; }

static void begin_scroll(MenuUi *ui, uint32_t now)
{
  ui->scroll_from = ui->selected;
  ui->selected = (uint8_t)((ui->selected + 1U) % MENU_ITEM_COUNT);
  ui->view = MENU_SCROLL;
  ui->view_start_ms = now;
  ui->dirty = 1U;
}

void MenuUi_Init(MenuUi *ui)
{
  memset(ui, 0, sizeof(*ui));
  ui->view = MENU_VIEW;
  ui->dirty = 1U;
}

void MenuUi_Next(MenuUi *ui, uint32_t now)
{
  if (ui->view == MENU_PREVIEW) return;
  if (ui->view == MENU_SCROLL) {
    /* Avoid losing ordinary repeated taps while the animation completes. */
    if (ui->queued_steps < 250U) ++ui->queued_steps;
  } else begin_scroll(ui, now);
}

void MenuUi_Confirm(MenuUi *ui, uint32_t now)
{
  if (ui->view == MENU_PREVIEW) return;
  ui->queued_steps = 0U;
  ui->view = MENU_PREVIEW;
  ui->view_start_ms = now;
  ++ui->preview_count;
  ui->dirty = 1U;
}

void MenuUi_Tick(MenuUi *ui, uint32_t now)
{
  if (ui->view == MENU_PREVIEW && now - ui->view_start_ms >= MENU_PREVIEW_MS) {
    /* The chosen index survives completion; real tasks can use this return. */
    ui->view = MENU_VIEW;
    ++ui->completion_count;
    ui->dirty = 1U;
  } else if (ui->view == MENU_SCROLL && now - ui->view_start_ms >= MENU_SCROLL_MS) {
    ui->view = MENU_VIEW;
    ui->dirty = 1U;
    if (ui->queued_steps) { --ui->queued_steps; begin_scroll(ui, now); }
  }
}

static void draw_label(uint8_t index, int16_t center_y, uint8_t scale)
{
  const char *text = MenuUi_Label(index);
  int16_t width = (int16_t)(strlen(text) * 6U * scale - scale);
  int16_t height = (int16_t)(7U * scale);
  OLED_Draw_Big_String((int16_t)((128 - width) / 2),
                      (int16_t)(center_y - height / 2), scale, text);
}

void MenuUi_Draw(const MenuUi *ui, uint32_t now)
{
  OLED_Clear();
  if (ui->view == MENU_PREVIEW) {
    draw_label(ui->selected, 31, 5U);
  } else {
    if (ui->view == MENU_SCROLL) {
      uint32_t elapsed = now - ui->view_start_ms;
      if (elapsed > MENU_SCROLL_MS) elapsed = MENU_SCROLL_MS;
      int16_t offset = (int16_t)((elapsed * 23U) / MENU_SCROLL_MS);
      uint8_t halfway = elapsed >= MENU_SCROLL_MS / 2U;
      /* Remove the former top row immediately. These are the only three
       * labels drawn: former center, chosen former bottom, incoming bottom. */
      draw_label(ui->scroll_from, (int16_t)(32 - offset), halfway ? 2U : 3U);
      draw_label(ui->selected, (int16_t)(55 - offset), halfway ? 3U : 2U);
      draw_label((uint8_t)((ui->selected + 1U) % MENU_ITEM_COUNT),
                 (int16_t)(78 - offset), 2U);
    } else {
      draw_label((uint8_t)((ui->selected + MENU_ITEM_COUNT - 1U) % MENU_ITEM_COUNT), 9, 2U);
      draw_label(ui->selected, 32, 3U);
      draw_label((uint8_t)((ui->selected + 1U) % MENU_ITEM_COUNT), 55, 2U);
    }
    OLED_Draw_Rect(7, 20, 114, 25);
  }
  OLED_Refresh();
}
