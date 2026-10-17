static void start_run(void)
{
  uint32_t now=HAL_GetTick();
  if(auto_phase!=IDLE || reference_run_used || pivot_active){
    send_text("EVENT,REFUSE_START,once_per_reset_or_busy\r\n");return;
  }
  if(laser_status!=4U){
    send_text("EVENT,REFUSE_START,laser_init\r\n");return;
  }
  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_RESET ||
     !gray_frames || sample_age_ms(now,gray_last_ms)>200U ||
     !(gray_value&0x18U) || active_count(gray_value)>3U ||
     !imu_frames || sample_age_ms(now,imu_last_ms)>100U){
    send_text("EVENT,REFUSE_START,key_line_or_imu\r\n");return;
  }
  reference_run_used=1U;reset_encoders();
  turns=center_hits=feature_hits=line_lost=0U;feature_candidate=FEATURE_LINE;
  search_plan_reset();reference_next=reference_halted=0U;garage_type=GARAGE_UNKNOWN;
  auto_phase=SEARCH;state=FOLLOW;
  run_start_ms=state_start_ms=last_turn_complete_ms=watch_ms=now;
  watch_position=0;
  send_text("EVENT,START,EXT2_LASER_SEARCH_UNTESTED,CLOCKWISE,NO_VISION\r\n");
  record_corner_event("ORIGIN",0U,NO_TURN,now);
}
