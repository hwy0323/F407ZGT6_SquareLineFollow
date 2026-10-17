#ifndef ALL_MENU_APP_H
#define ALL_MENU_APP_H
#include <stdint.h>
#include "mv_client.h"
#define ALL_MENU_COUNT 11U
typedef enum { APP_MENU,APP_POSE,APP_VISION,APP_PREFLIGHT,APP_RUNNING,APP_FINISH } AppPhase;
typedef struct {
  AppPhase phase;
  uint8_t selected,visual,dirty,success,stable,rebooted;
  uint32_t since,stable_since,run_since,rx_errors,rx_overflows;
  MvClient vision;MvEvent result;
  char message[80];
} MenuApp;
void menu_app_init(MenuApp *app);
void menu_app_next(MenuApp *app);
void menu_app_start(MenuApp *app,uint32_t now);
void menu_app_stop(MenuApp *app,const char *reason,uint32_t now);
void menu_app_tick(MenuApp *app,uint32_t now);
const char *menu_app_label(uint8_t selected);
uint8_t menu_app_laser_on(const MenuApp *app);
/* 少量硬件接口；主机测试可替换，任务逻辑不依赖 HAL。 */
uint32_t app_now(void);
void app_log(const char *line);
void app_force_stop(void);
void app_reboot(void);
uint8_t app_screen_ready(void);
#endif
