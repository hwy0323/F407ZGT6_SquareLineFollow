#ifndef MENU_KEYS_H
#define MENU_KEYS_H
#include <stdint.h>
typedef enum { KEY_NONE,KEY_NEXT,KEY_START,KEY_STOP } KeyEvent;
typedef struct {uint8_t raw,stable,ready,pending;uint32_t changed,pressed;} MenuKey;
KeyEvent menu_key_tick(MenuKey *key,uint8_t index,uint8_t high,uint8_t active,uint32_t now);
#endif
