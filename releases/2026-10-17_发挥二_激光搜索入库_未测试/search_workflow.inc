static void combined_abort(const char *why)
{
  if(pivot_active)reference_finish(why);
  stop_motors();reference_run_used=reference_halted=1U;state=STOPPED;auto_phase=FAILED;
  char line[140];snprintf(line,sizeof(line),"EVENT,EXT2_STOP,%s\r\n",why);send_text(line);
}
static void search_next_motion(uint32_t now)
{
  if(now-pivot_last_stop_ms<STEP_GAP_MS)return;
  reference_start(now);
  if(!pivot_active)combined_abort("MOTION_START_REFUSED");
}
static void search_safety(uint32_t now)
{
  if(auto_phase<=IDLE || auto_phase>=DONE)return;
  if(now-run_start_ms>JOB_MAX_MS)combined_abort("JOB_TIMEOUT");
  else if(!imu_frames || sample_age_ms(now,imu_last_ms)>150U)combined_abort("IMU_STALE");
}
static void search_measure(uint32_t now)
{
  stop_motors(); /* I2C polling must NEVER run while a motor is enabled. */
  if(now-phase_ms>MEASURE_TIMEOUT_MS){combined_abort("LASER_TIMEOUT_OR_INVALID");return;}
  if(laser_status!=4U){combined_abort("LASER_API_ERROR");return;}
  if(now-last_range_poll<20U)return;
  last_range_poll=now;LaserTest_Process();
  if(laser_status!=4U){combined_abort("LASER_API_ERROR");return;}
  if(laser_sample_count==range_seen)return;
  range_seen=laser_sample_count;
  char line[140];
  snprintf(line,sizeof(line),"LASER,%lu,mm=%u,status=%u,discard=%u\r\n",
           (unsigned long)range_seen,laser_distance_mm,laser_range_status,range_discard);send_text(line);
  /* Discard old/pipeline readings acquired before stopping at the probe point. */
  if(range_discard){range_discard--;return;}
  if(laser_range_status!=0U || laser_distance_mm<20U || laser_distance_mm>2000U)return;
  range_samples[range_used++]=laser_distance_mm;
  if(range_used<RANGE_SAMPLE_COUNT)return;
  garage_type=search_classify(range_samples,LASER_RIGHT_OFFSET_MM,&range_median);
  snprintf(line,sizeof(line),"EVENT,TYPE,%s,median=%u,depth_est=%ld,offset=%d\r\n",
    garage_type==GARAGE_REVERSE?"REVERSE":garage_type==GARAGE_SIDE?"SIDE":"UNKNOWN",
    range_median,(long)(range_median+LASER_RIGHT_OFFSET_MM),LASER_RIGHT_OFFSET_MM);send_text(line);
  if(garage_type==GARAGE_UNKNOWN){combined_abort("LASER_TYPE_UNKNOWN");return;}
  if(!search_plan_return(reference_progress)){combined_abort("PROBE_DISTANCE_ERROR");return;}
  auto_phase=PROBE_BACK;search_next_motion(HAL_GetTick());
}
static void search_tick(uint32_t now)
{
  if(auto_phase==LINE_PAUSE){
    stop_motors();
    if(now-phase_ms<200U)return;
    garage_origin[0]=enc_count(&enc_a);garage_origin[1]=enc_count(&enc_b);
    garage_origin[2]=enc_count(&enc_c);garage_yaw=imu_yaw_x100;
    auto_phase=PROBE_FORWARD;search_next_motion(now);return;
  }
  if(auto_phase==MEASURE){search_measure(now);return;}
  if(auto_phase!=PROBE_FORWARD && auto_phase!=PROBE_BACK && auto_phase!=ENTRY)return;
  if(reference_halted){combined_abort("MOTION_FAILED");return;}
  if(pivot_active)return;
  if(auto_phase==PROBE_FORWARD){
    if(reference_next==0U){search_next_motion(now);return;}
    if(now-pivot_last_stop_ms<STEP_GAP_MS)return;
    auto_phase=MEASURE;phase_ms=now;range_used=0;range_discard=3;
    range_seen=laser_sample_count;last_range_poll=now;
    send_text("EVENT,PROBE_STOPPED_MEASURE\r\n");return;
  }
  if(auto_phase==PROBE_BACK){
    if(reference_next<2U){search_next_motion(now);return;}
    int32_t residual=reference_encoder_progress(
      (int16_t)(enc_count(&enc_a)-garage_origin[0]),
      (int16_t)(enc_count(&enc_b)-garage_origin[1]),
      (int16_t)(enc_count(&enc_c)-garage_origin[2]),1);
    if(reference_abs(residual)>40 || reference_abs(reference_yaw_delta(garage_yaw,imu_yaw_x100))>200){
      combined_abort("PROBE_RETURN_POSITION_ERROR");return;
    }
    if(!search_plan_entry(garage_type)){combined_abort("ENTRY_TYPE_INVALID");return;}
    auto_phase=ENTRY;send_text("EVENT,ENTRY_BEGIN,UNTESTED\r\n");
  }
  if(auto_phase==ENTRY){
    if(reference_next==ENTRY_STEP_COUNT){
      stop_motors();auto_phase=DONE;state=STOPPED;
      send_text("EVENT,EXT2_ENTRY_DONE,UNTESTED,motors_off\r\n");return;
    }
    search_next_motion(now);
  }
}
