#include "../reference_steps.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct { int16_t count; } Timer;
static Timer enc_a, enc_b, enc_c, enc_d;
static uint8_t pivot_active, fake_key = 1U;
static uint16_t pivot_start_yaw, imu_yaw_x100;
static uint32_t pivot_last_stop_ms, pivot_start_ms, imu_last_ms, imu_frames;
static int16_t pivot_encoder_start[4], left_command, right_command;
static uint8_t pivot_rx_buffer[16], pivot_rx_tail, pivot_rx_head;
static uint32_t fake_now;
static char last_log[256];
#define GPIOA 0
#define GPIO_PIN_15 15
#define GPIO_PIN_RESET 0
static uint32_t HAL_GetTick(void) { return fake_now; }
static int HAL_GPIO_ReadPin(int a, int b) { (void)a; (void)b; return fake_key; }
static uint32_t sample_age_ms(uint32_t now, uint32_t stamp) { return now - stamp; }
static int16_t enc_count(Timer *timer) { return timer->count; }
static int16_t clamp(int16_t v, int16_t lim) { return v > lim ? lim : v < -lim ? -lim : v; }
static void stop_motors(void) { left_command = right_command = 0; }
static void drive(int16_t a, int16_t b) { left_command = a; right_command = b; }
static void send_text(const char *s) { (void)snprintf(last_log, sizeof(last_log), "%s", s); }
#include "../reference_motion.inc"
enum { STOPPED, FOLLOW };
static int state;
static uint8_t auto_phase;
static uint32_t auto_pause_start_ms;
static void stop_run(const char *s) { (void)s; state = STOPPED; stop_motors(); }
#include "../combined_workflow.inc"
static void reset_test(void)
{
  reference_next = reference_running = reference_phase = reference_halted = 0U;
  reference_anchor_set = pivot_active = 0U;
  pivot_last_stop_ms = 0U;
  fake_now = imu_last_ms = 1000U; imu_frames = 1U; imu_yaw_x100 = 10000U;
  enc_a.count = enc_b.count = enc_c.count = enc_d.count = 0;
  pivot_rx_head = pivot_rx_tail = 0U;
  fake_key = 1U; auto_phase = 0U; state = STOPPED; auto_pause_start_ms = 0U;
  stop_motors(); last_log[0] = 0;
}
static void tick(uint32_t now, uint16_t yaw)
{ fake_now = imu_last_ms = now; imu_yaw_x100 = yaw; reference_control(now); }
static void endpoint_test(uint8_t index, int32_t offset, uint8_t allowed)
{
  reset_test(); reference_next=index; reference_start(1000);
  reference_phase=1U; reference_phase_ms=1000;
  const ReferenceStep *s=&reference_steps[index];
  int32_t actual=s->target+offset; uint16_t yaw=pivot_start_yaw;
  if(s->direction=='L') yaw=(uint16_t)((yaw+36000-actual)%36000);
  else if(s->direction=='R') yaw=(uint16_t)((yaw+actual)%36000);
  else {int sign=s->direction=='F'?1:-1;
    enc_a.count=enc_b.count=(int16_t)(-sign*actual);enc_c.count=(int16_t)(sign*actual);}
  tick(1200,yaw);
  assert(!pivot_active&&!left_command&&!right_command);
  assert(reference_halted==!allowed);
  assert(strstr(last_log,allowed?",DONE_OFFSET,":",END_ERROR,"));
  if(!allowed){auto_phase=3U;combined_tick(1350);assert(auto_phase==4U&&!pivot_active);}
}
int main(void)
{
  endpoint_test(0,100,1); endpoint_test(0,101,0);
  endpoint_test(1,40,1); endpoint_test(1,41,0);
  endpoint_test(2,250,1); endpoint_test(2,251,0);
  endpoint_test(3,-40,1); endpoint_test(3,-41,0);
  endpoint_test(4,250,1); endpoint_test(4,251,0);
  assert(reference_steps[3].direction == 'B');
  assert(reference_steps[3].target == 1146 + 201);
  assert(reference_steps[3].target - 1318 == 29);
  assert(strcmp(reference_steps[3].command, "@REF4:B1347") == 0);
  ReferenceParser parser = {0};
  const char *old_reverse = "@REF4:B1232\n";
  while (*old_reverse) assert(!reference_feed(&parser, (uint8_t)*old_reverse++, 3U));
  const char *previous_reverse = "@REF4:B1318\n";
  while (*previous_reverse) assert(!reference_feed(&parser, (uint8_t)*previous_reverse++, 3U));
  const char *new_reverse = "@REF4:B1347\n"; int accepted = 0;
  while (*new_reverse) accepted += reference_feed(&parser, (uint8_t)*new_reverse++, 3U);
  assert(accepted == 1);
  reset_test(); reference_start(1000); tick(1000, 10000);
  assert(left_command == -200 && right_command == 200);
  tick(1010, 10000); assert(left_command == -200); /* not a 10ms pulse */
  tick(1030, 9800); assert(left_command == -135);
  tick(1040, 9600); assert(!left_command && reference_phase == 1U);
  tick(1240, 9454); assert(!pivot_active && strstr(last_log, ",DONE,"));
  reference_report(1240); assert(strstr(last_log, "REF,"));
  pivot_rx_buffer[0] = 'G'; pivot_rx_head = 1; reference_read_dap(1240); assert(!pivot_active);

  reset_test(); reference_start(1000); tick(1000, 10000); tick(1040, 9600);
  tick(1240, 9300); assert(!pivot_active && reference_halted && !left_command && !right_command);
  assert(strstr(last_log, ",END_ERROR,"));
  tick(1260, 9400); tick(1460, 9454);
  assert(!pivot_active && !left_command && !right_command);
  reference_start(1500); assert(!pivot_active); /* no re-drive after overshoot */

  reset_test(); reference_next = 1U; reference_start(1000); tick(1000, 10000);
  assert(left_command == 180 && right_command == 180);
  enc_a.count = enc_b.count = -100; enc_c.count = 100;
  tick(1050, 10000); assert(left_command == 180 && right_command == 180);
  enc_a.count = enc_b.count = -2145; enc_c.count = 2145;
  tick(2500, 10000); assert(reference_phase == 1U && !left_command);
  enc_a.count = enc_b.count = -2194; enc_c.count = 2194;
  tick(2700, 10000); assert(!pivot_active && strstr(last_log, ",DONE,F,"));

  reset_test(); reference_start(1000); reference_phase = 1U; reference_phase_ms = 1000;
  tick(1200, 10000); assert(!pivot_active && strstr(last_log, ",END_ERROR,"));
  assert(!left_command && !right_command && reference_halted);
  reset_test(); reference_next = 3U; reference_start(1000); tick(1000,10000);
  assert(left_command == -180 && right_command == -180);
  enc_a.count=enc_b.count=1305; enc_c.count=-1305;
  tick(2000,10000); tick(2010,10000);
  assert(reference_phase==1U && !left_command && !right_command);
  enc_a.count=enc_b.count=1347; enc_c.count=-1347;
  tick(2210,10000); assert(!pivot_active && strstr(last_log, ",DONE,B,"));
  reset_test(); reference_start(1000); fake_key = 0; tick(1010, 10000);
  assert(!pivot_active && strstr(last_log, ",KEY,"));
  reset_test(); reference_start(1000); fake_now = 1200; reference_control(1200);
  assert(!pivot_active && strstr(last_log, ",IMU_STALE,"));
  reset_test(); reference_start(1000); tick(1800, 10000);
  assert(!pivot_active && strstr(last_log, ",NO_YAW_PROGRESS,"));

  reset_test(); auto_phase = 2U; tick(1200, 10000); combined_tick(1200);
  assert(pivot_active && auto_phase == 3U);
  uint32_t now = 1300;
  const int32_t field_offsets[5]={69,34,95,16,209};
  for (uint8_t i = 0; i < 5; ++i) {
    const ReferenceStep *s = &reference_steps[i];
    assert(reference_running == i && pivot_active);
    uint16_t yaw = pivot_start_yaw;
    int32_t actual=s->target+field_offsets[i];
    if (s->direction == 'L') yaw = (uint16_t)((yaw + 36000 - actual) % 36000);
    else if (s->direction == 'R') yaw = (uint16_t)((yaw + actual) % 36000);
    else {
      int32_t sign = s->direction == 'F' ? 1 : -1;
      enc_a.count = (int16_t)(pivot_encoder_start[0] - sign * actual);
      enc_b.count = (int16_t)(pivot_encoder_start[1] - sign * actual);
      enc_c.count = (int16_t)(pivot_encoder_start[2] + sign * actual);
    }
    tick(now, yaw); tick(now + 200, yaw); combined_tick(now + 200);
    assert(!pivot_active&&!reference_halted);
    if (i < 4) {
      tick(now + 349, yaw); combined_tick(now + 349); assert(!pivot_active);
      tick(now + 350, yaw); combined_tick(now + 350); assert(pivot_active);
    }
    now += 450;
  }
  assert(auto_phase == 4U && strstr(last_log, "SIDE_REFERENCE_DONE"));
  reset_test(); auto_phase = 3U; combined_abort("REMOTE_X"); combined_tick(2000);
  assert(auto_phase == 4U && !pivot_active);
  assert(!combined_start_feed('G'));
  const char *cmd = "@SIDE:RUN\n"; int start_count = 0;
  while (*cmd) start_count += combined_start_feed((uint8_t)*cmd++);
  assert(start_count == 1);
  puts("PASS: logged fine/accepted offset distinction, boundary aborts, no pulses, five-step field-offset joins, safety stop");
  return 0;
}
