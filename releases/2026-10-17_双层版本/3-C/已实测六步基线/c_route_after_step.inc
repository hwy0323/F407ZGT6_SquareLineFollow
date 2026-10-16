static void route_after_step(const char *reason)
{
  char line[100];
  if(!route_active)return;
  uint8_t final_step=(size_t)(route_index+1U)==ROUTE_STEP_COUNT;
  uint8_t accepted=strcmp(reason,"DONE")==0;
  if(final_step) accepted |= strcmp(reason,"DONE_OFFSET")==0;
  else accepted |= strcmp(reason,"OVERSHOOT")==0 || strcmp(reason,"DISTANCE_OVERSHOOT")==0;
  if(!accepted){route_abort(reason);return;}
  if(route_index==4U){
    send_text("EVENT,EXIT_FINISHED,5\r\n");
    send_text("EVENT,EXIT_DONE,prepare_forward_50mm_then_stop_C\r\n");
  }
  if(final_step){
    route_active=route_waiting=0U;auto_phase=4U;
    stop_motors();state=STOPPED;
    (void)snprintf(line,sizeof(line),"EVENT,ROUTE_DONE,%lu,%u\r\n",
      (unsigned long)(HAL_GetTick()-route_start_ms),(unsigned int)ROUTE_STEP_COUNT);
    send_text(line);
    (void)snprintf(line,sizeof(line),"EVENT,STOP,%lu,C_REACHED_FROM_BC_GARAGE,turns=%u\r\n",
      (unsigned long)(HAL_GetTick()-route_start_ms),(unsigned int)turns);
    send_text(line);
    return;
  }
  route_index++;route_waiting=1U;
  (void)snprintf(line,sizeof(line),"EVENT,ROUTE_NEXT,%u\r\n",(unsigned int)(route_index+1U));
  send_text(line);
}
