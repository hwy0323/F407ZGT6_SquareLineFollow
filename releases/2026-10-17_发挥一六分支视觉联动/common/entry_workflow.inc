static void combined_abort(const char *reason)
{
  char line[120];reference_run_used=1U;
  if(pivot_active)reference_finish(reason);
  if(state!=STOPPED)stop_run(reason);
  reference_halted=1U;auto_phase=4U;stop_motors();
  (void)snprintf(line,sizeof(line),"EVENT,TASK4_ABORT,%s,%s\r\n",ENTRY_EDGE,reason);
  send_text(line);
}
static uint8_t transit_lines_complete(void)
{
  if(!task4_plan_valid(&parking_plan) || parking_events!=ENTRY_PASS_LINES)return 0U;
  for(uint8_t edge=1U;edge<ENTRY_REQUIRED_CORNERS;edge++)
    if(parking_lines_on_edge[edge]!=2U)return 0U;
  return 1U;
}
static void combined_tick(uint32_t now)
{
  char line[140];
  if(auto_phase==2U && now-auto_pause_start_ms>=ENTRY_WAIT_MS){
    if(!entry_confirmed || turns!=ENTRY_REQUIRED_CORNERS || !transit_lines_complete()){
      combined_abort("WRONG_EDGE_OR_GARAGE_SEQUENCE");return;
    }
    auto_phase=3U;
    (void)snprintf(line,sizeof(line),"EVENT,TASK4_ENTRY_BEGIN,%s,%s,branch=%u\r\n",
      ENTRY_EDGE,parking_plan.reverse?"REVERSE":"SIDE",parking_plan.branch_id);
    send_text(line);reference_start(now);
    if(!pivot_active)combined_abort("FIRST_STEP_REFUSED");
    return;
  }
  if(auto_phase!=3U)return;
  if(reference_halted){combined_abort("REFERENCE_STEP_FAILED");return;}
  if(pivot_active)return;
  if(reference_next==ENTRY_STEP_COUNT){
    stop_motors();auto_phase=4U;entry_finished=1U;
    (void)snprintf(line,sizeof(line),"EVENT,TASK4_ENTRY_DONE,%s,%s,motors_off\r\n",
      ENTRY_EDGE,parking_plan.reverse?"REVERSE":"SIDE");
    send_text(line);return;
  }
  if(now-pivot_last_stop_ms>=150U){
    reference_start(now);
    if(!pivot_active)combined_abort("NEXT_STEP_REFUSED");
  }
}
