#include "keys.h"
KeyEvent menu_key_tick(MenuKey *key,uint8_t index,uint8_t high,uint8_t active,uint32_t now)
{
  if(high!=key->raw){key->raw=high;key->changed=now;}
  if(now-key->changed<30U)return KEY_NONE;
  /* 上电按住或复位时仍按住的键，必须松开后才能重新使用。 */
  if(!key->ready){if(high){key->ready=1;key->stable=1;}return KEY_NONE;}
  if(high==key->stable)return KEY_NONE;
  key->stable=high;
  if(!high){
    key->pressed=now;key->pending=active?0U:1U;
    return active?KEY_STOP:KEY_NONE;
  }
  if(!key->pending)return KEY_NONE;
  key->pending=0;
  /* PA15 始终只切题，不论按多久；PE6 才能确认启动。 */
  return index==0U?KEY_NEXT:KEY_START;
}
