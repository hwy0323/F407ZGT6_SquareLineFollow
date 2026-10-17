/* One bounded motion at a time; workflow advances only after an accepted stop. */
#include "../entry_plan.h"
static uint8_t reference_next, reference_running, reference_phase;
static uint8_t reference_halted;
static uint32_t reference_phase_ms;
static int32_t reference_progress;
static uint32_t reference_watch_ms;
static int32_t reference_watch_progress;
static uint32_t reference_rate_ms;
static int32_t reference_rate_progress, reference_rate;

static int32_t reference_abs(int32_t value) { return value < 0 ? -value : value; }

static int32_t reference_progress_now(void)
{
  const ReferenceStep *step = &reference_steps[reference_running];
  if (step->direction == 'L' || step->direction == 'R')
    return (step->direction == 'L' ? -1 : 1) *
           reference_yaw_delta(pivot_start_yaw, imu_yaw_x100);
  return reference_encoder_progress(
    (int16_t)(enc_count(&enc_a) - pivot_encoder_start[0]),
    (int16_t)(enc_count(&enc_b) - pivot_encoder_start[1]),
    (int16_t)(enc_count(&enc_c) - pivot_encoder_start[2]),
    step->direction == 'F' ? 1 : -1);
}

static void reference_finish(const char *reason)
{
  char line[160];
  stop_motors();
  pivot_active = 0U;
  pivot_last_stop_ms = HAL_GetTick();
  if (strcmp(reason, "DONE") != 0 && strcmp(reason, "DONE_OFFSET") != 0) reference_halted = 1U;
  reference_progress = reference_progress_now();
  (void)snprintf(line, sizeof(line),
    "EVENT,REF_END,%u,%s,%c,target=%ld,actual=%ld,yaw_start=%u,yaw_end=%u\r\n",
    reference_running + 1U, reason, reference_steps[reference_running].direction,
    (long)reference_steps[reference_running].target, (long)reference_progress,
    pivot_start_yaw, imu_yaw_x100);
  send_text(line);
}

static void reference_start(uint32_t now)
{
  char line[130];
  if (reference_halted || pivot_active || reference_next >= ENTRY_STEP_COUNT ||
      (pivot_last_stop_ms && now - pivot_last_stop_ms < 150U) ||
      !imu_frames || sample_age_ms(now, imu_last_ms) > 100U ||
      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
    send_text("EVENT,REF_REFUSE,check_idle_imu_key_or_sequence\r\n"); return;
  }
  reference_running = reference_next++;
  pivot_start_yaw = imu_yaw_x100;
  pivot_encoder_start[0] = enc_count(&enc_a);
  pivot_encoder_start[1] = enc_count(&enc_b);
  pivot_encoder_start[2] = enc_count(&enc_c);
  pivot_encoder_start[3] = enc_count(&enc_d);
  pivot_start_ms = reference_phase_ms = now;
  reference_watch_ms = now;
  reference_watch_progress = 0;
  reference_phase = 0U;
  reference_progress = 0;
  reference_rate_ms = now;
  reference_rate_progress = reference_progress_now();
  reference_rate = 0;
  pivot_active = 1U;
  (void)snprintf(line, sizeof(line), "EVENT,REF_START,%u,%c,%ld,yaw=%u\r\n",
    reference_running + 1U, reference_steps[reference_running].direction,
    (long)reference_steps[reference_running].target, pivot_start_yaw);
  send_text(line);
}

/* Continuous travel with one coast cutoff; never restart for endpoint trim.
 * Rate is measured from progress/time; no assumption about gyro raw units.
 */
static void reference_control(uint32_t now)
{
  if (!pivot_active) return;
  const ReferenceStep *step = &reference_steps[reference_running];
  uint8_t linear = step->direction == 'F' || step->direction == 'B';
  int32_t tolerance = linear ? 20 : 60;
  /* Field-accepted loaded-car offsets. Keep fine accuracy distinct in logs. */
  int32_t accepted_offset = linear ? 40 : (step->target < 1500 ? 100 : 250);
  reference_progress = reference_progress_now();
  int32_t error = step->target - reference_progress;
  int32_t yaw = reference_yaw_delta(pivot_start_yaw, imu_yaw_x100);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
    reference_finish("KEY"); return;
  }
  if (!imu_frames || sample_age_ms(now, imu_last_ms) > 150U) {
    reference_finish("IMU_STALE"); return;
  }
  if (now - pivot_start_ms > (linear ? 20000U : 12000U)) {
    reference_finish("TIMEOUT"); return;
  }
  if ((linear && reference_abs(yaw) > 800) ||
      reference_progress < (linear ? -100 : -300) ||
      reference_progress > step->target + (linear ? 350 : 800)) {
    reference_finish("MOTION_LIMIT"); return;
  }
  uint32_t dt = now - reference_rate_ms;
  if (dt >= 10U) {
    int32_t measured_rate = (reference_progress - reference_rate_progress) * 1000 / (int32_t)dt;
    reference_rate = (reference_rate * 2 + measured_rate) / 3;
    reference_rate_progress = reference_progress;
    reference_rate_ms = now;
  }
  if (reference_phase == 1U) {
    stop_motors();
    if (now - reference_phase_ms < 200U) return;
    if (reference_abs(error) <= tolerance) { reference_finish("DONE"); return; }
    if (reference_abs(error) <= accepted_offset) { reference_finish("DONE_OFFSET"); return; }
    reference_finish("END_ERROR"); return;
  }
  if (now - reference_watch_ms >= 800U) {
    if (reference_progress - reference_watch_progress < (linear ? 8 : 20)) {
      reference_finish(linear ? "ENCODER_STALL" : "NO_YAW_PROGRESS"); return;
    }
    reference_watch_ms = now;
    reference_watch_progress = reference_progress;
  }
  /* Estimate coast travel from the recent measured rate. Caps bound the lead. */
  /* Recent loaded-car linear runs coasted farther than the old 60ms model. */
  int32_t lead = reference_rate > 0 ? reference_rate * (linear ? 110 : 50) / 1000 : 0;
  if (lead < tolerance) lead = tolerance;
  if (lead > (linear ? 250 : 650)) lead = linear ? 250 : 650;
  if (error <= lead) {
    stop_motors(); reference_phase = 1U; reference_phase_ms = now; return;
  }
  if (linear) {
    int16_t speed = error > 250 ? 180 : 135;
    int16_t correction = clamp((int16_t)(yaw / 10), 30);
    if (step->direction == 'B') speed = -speed;
    drive(speed - correction, speed + correction);
  } else {
    int16_t speed = error > 1500 || now - pivot_start_ms < 30U ? 200 : 135;
    if (step->direction == 'L') speed = -speed;
    drive(speed, -speed);
  }
}

static void reference_report(uint32_t now)
{
  char line[210];
  const ReferenceStep *step = &reference_steps[reference_running];
  (void)snprintf(line, sizeof(line),
    "REF,%lu,%u,%u,%c,%ld,%ld,%u,%u,%d,%d,%d,%d,%d,%d,%lu,%lu\r\n",
    (unsigned long)(pivot_active ? now - pivot_start_ms : 0U), pivot_active,
    reference_next, step->direction, (long)step->target,
    (long)(reference_next ? reference_progress_now() : 0), pivot_start_yaw, imu_yaw_x100,
    left_command, right_command,
    (int)(int16_t)(enc_count(&enc_a) - pivot_encoder_start[0]),
    (int)(int16_t)(enc_count(&enc_b) - pivot_encoder_start[1]),
    (int)(int16_t)(enc_count(&enc_c) - pivot_encoder_start[2]),
    (int)(int16_t)(enc_count(&enc_d) - pivot_encoder_start[3]),
    (unsigned long)(imu_frames ? sample_age_ms(now, imu_last_ms) : 9999U),
    (unsigned long)imu_frames);
  send_text(line);
}
