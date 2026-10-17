#include "native.h"
#include <string.h>
static char last_event[220];
static NativeId current=NATIVE_COUNT;
static uint8_t completed;
void completion_select(NativeId id){current=id;completed=0U;last_event[0]=0;}
void native_event(const char *text)
{
  if(strncmp(text,"EVENT,",6))return;
  if(strstr(text,"STOP") || strstr(text,"ABORT") || strstr(text,"REFUSE") ||
     strstr(text,"DONE") || strstr(text,"FINISHED")){
    size_t n=strlen(text);if(n>=sizeof(last_event))n=sizeof(last_event)-1U;
    memcpy(last_event,text,n);last_event[n]=0;
  }
  /* 只有原控制器的正常完成事件才是 DONE，丢线／里程上限不能冒充到点。 */
  const char *token=0;
  switch(current){
    case NATIVE_11:token=",B_REACHED,";break;
    case NATIVE_12:token=",A_REACHED_ONE_LAP,";break;
    case NATIVE_21:token="EVENT,TASK2_1_DONE,";break;
    case NATIVE_22:token="EVENT,SIDE_REFERENCE_DONE,";break;
    case NATIVE_3A:token=",A_REACHED_FROM_BC_GARAGE,";break;
    case NATIVE_3B:token=",B_REACHED_FROM_BC_GARAGE,";break;
    case NATIVE_3C:token=",C_REACHED_FROM_BC_GARAGE,";break;
    case NATIVE_3D:token=",D_REACHED_FROM_BC_GARAGE,";break;
    case NATIVE_5:token="EVENT,EXT2_ENTRY_DONE,";break;
    default:break;
  }
  if(token && strstr(text,token))completed=1U;
}
uint8_t completion_done(void){return completed;}
const char *native_last_event(void){return last_event;}
