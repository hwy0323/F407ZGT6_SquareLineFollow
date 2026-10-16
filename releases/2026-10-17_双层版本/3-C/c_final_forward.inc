/* Merged final F250: one continuous move, same signed-median / IMU
 * coast-stop controller as the accepted final F50. No endpoint pulses. */
static uint8_t c_final_stopping;
static uint32_t c_final_stop_ms, c_final_rate_ms, c_final_watch_ms;
static int32_t c_final_rate_counts, c_final_rate, c_final_watch_counts;
static void c_final_begin(uint32_t now)
{
  c_final_stopping=0U;
  c_final_stop_ms=c_final_rate_ms=c_final_watch_ms=now;
  c_final_rate_counts=c_final_rate=c_final_watch_counts=0;
}
static int32_t c_final_counts(void)
{
  int32_t v[3]={-(int16_t)(enc_count(&enc_a)-pivot_encoder_start[0]),
                -(int16_t)(enc_count(&enc_b)-pivot_encoder_start[1]),
                 (int16_t)(enc_count(&enc_c)-pivot_encoder_start[2])};
  for(uint8_t i=0U;i<3U;i++)
    for(uint8_t j=i+1U;j<3U;j++)
      if(v[j]<v[i]){int32_t temp=v[i];v[i]=v[j];v[j]=temp;}
  return v[1];
}
static void c_final_control(uint32_t now)
{
  int32_t progress=c_final_counts();
  int32_t error=linear_target_counts-progress;
  int32_t yaw=pivot_delta_x100();
  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_RESET){pivot_stop("KEY");return;}
  if(!imu_frames || sample_age_ms(now,imu_last_ms)>150U){pivot_stop("IMU_STALE");return;}
  if(now-pivot_start_ms>8000U){pivot_stop("TIMEOUT");return;}
  if(yaw>800 || yaw< -800 || progress< -100 || progress>linear_target_counts+80){
    pivot_stop("MOTION_LIMIT");return;
  }
  uint32_t dt=now-c_final_rate_ms;
  if(dt>=10U){
    int32_t measured=(progress-c_final_rate_counts)*1000/(int32_t)dt;
    c_final_rate=(c_final_rate*2+measured)/3;
    c_final_rate_counts=progress;c_final_rate_ms=now;
  }
  if(c_final_stopping){
    stop_motors();
    if(now-c_final_stop_ms<200U)return;
    int32_t deviation=error<0?-error:error;
    pivot_stop(deviation<=20?"DONE":deviation<=40?"DONE_OFFSET":"END_ERROR");
    return;
  }
  if(now-c_final_watch_ms>=800U){
    if(progress-c_final_watch_counts<8){pivot_stop("ENCODER_STALL");return;}
    c_final_watch_counts=progress;c_final_watch_ms=now;
  }
  int32_t lead=c_final_rate>0?c_final_rate*110/1000:0;
  if(lead<20)lead=20;
  if(lead>250)lead=250;
  if(error<=lead){stop_motors();c_final_stopping=1U;c_final_stop_ms=now;return;}
  int16_t speed=error>250?180:135;
  int16_t correction=clamp((int16_t)(yaw/10),30);
  drive(speed-correction,speed+correction);
}
