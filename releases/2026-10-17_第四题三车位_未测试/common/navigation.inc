static int16_t git_line_error(uint8_t bits)
{
  const int8_t weights[8] = {7, 5, 3, 1, -1, -3, -5, -7};
  int16_t sum = 0;
  uint8_t count = 0U;
  for (uint8_t i = 0U; i < 8U; i++)
    if (bits & (1U << i)) { sum += weights[i]; count++; }
  return count ? sum / count : 0;
}

static Direction git_detect_corner(uint8_t bits)
{
  uint8_t left = active_count(bits & 0xF0U);
  uint8_t right = active_count(bits & 0x0FU);
  if (left == 4U && right <= 1U) return LEFT_TURN;
  if (right == 4U && left <= 1U) return RIGHT_TURN;
  return NO_TURN;
}

/* A encoder was negative on all eight corners in the verified two-lap run.
 * These are the measured corner START positions, not inferred lap lengths.
 */
/* Preserve the measured, successfully tested square-follow corner windows.
 * The first bend at B appeared earlier than the rebased experimental window.
 */
static const int32_t true_corner_a_counts[CORNER_TARGET_COUNT] = {
  5100L, 11299L, 17567L, 23793L
};

static int32_t forward_a_counts(void)
{
  odom_update();
  return -odom_total[0];
}

static uint8_t near_true_corner(int32_t progress)
{
  int32_t target = true_corner_a_counts[turns];
  return progress >= target - CORNER_POSITION_WINDOW &&
         progress <= target + CORNER_POSITION_WINDOW;
}

/* A wide black feature in the middle of an edge cannot be a true corner.
 * This also covers extra tape on the otherwise plain CD and AB edges.
 */
static uint8_t in_straight_middle(int32_t progress)
{
  int32_t start = turns == 0U ? 200L :
                  true_corner_a_counts[turns - 1U] +
                    CORNER_POSITION_WINDOW;
  int32_t end = true_corner_a_counts[turns] -
                CORNER_POSITION_WINDOW;
  return progress > start && progress < end;
}

static void parking_event(const char *phase, uint32_t now,
                          uint8_t bits, int32_t progress)
{
  char line[220];
  static const char *edges[4]={"AB","BC","CD","DA"};
  odom_update();
  int32_t delta=(int32_t)imu_yaw_x100-reverse_start_yaw_x100;
  if(delta>18000)delta-=36000;
  if(delta< -18000)delta+=36000;
  (void)snprintf(line,sizeof(line),
    "EVENT,TASK4_PASS_GARAGE,%s,%u,%s,%lu,%02X,%ld,%u,%ld,%ld,%ld,%ld,%ld\r\n",
    phase,(unsigned int)parking_events,edges[turns<4U?turns:3U],
    (unsigned long)(now-run_start_ms),(unsigned int)bits,(long)progress,
    (unsigned int)imu_yaw_x100,(long)delta,
    (long)odom_total[0],(long)odom_total[1],(long)odom_total[2],(long)odom_total[3]);
  send_text(line);
}

static void git_verified_control(uint32_t now, uint8_t bits)
{
  Direction detected;
  int16_t correction;
  int32_t progress;
  uint8_t wide_black;
  uint8_t parking_zone;
  uint8_t left_black, right_black;
  if(turns>ENTRY_REQUIRED_CORNERS){stop_run("TARGET_EDGE_MISMATCH");return;}
  if(state==FOLLOW && turns==ENTRY_REQUIRED_CORNERS){
    progress=forward_a_counts();
    int32_t travel=progress-parking_corner_progress;
    if(progress>ENTRY_LINE_CENTER+ENTRY_LINE_WINDOW ||
       travel>4500L || now-last_turn_complete_ms>9000U){
      stop_run("TARGET_FIRST_LINE_MISSING");return;
    }
    if(progress< -200L || travel< -200L){stop_run("ODOM_DIRECTION");return;}
    if(bits==0U){
      if(!line_lost){line_lost=1U;line_lost_ms=now;}
      if(now-line_lost_ms>120U){stop_run("TARGET_SEARCH_LINE_LOST");return;}
      drive(110,110);return;
    }
    line_lost=0U;
    if(progress>=ENTRY_LINE_CENTER-ENTRY_LINE_WINDOW &&
       travel>=850L && now-last_turn_complete_ms>=500U &&
       map_garage_candidate(bits)){
      if(++parking_feature_hits>=3U){
        stop_motors();state=STOPPED;
        if(parking_events!=ENTRY_PASS_LINES){combined_abort("GARAGE_SEQUENCE_MISMATCH");return;}
        entry_confirmed=1U;auto_phase=2U;auto_pause_start_ms=now;
        char line[140];
        (void)snprintf(line,sizeof(line),
          "EVENT,TASK4_TARGET_LINE,%s,%lu,%ld,%02X,passed=%u\r\n",
          ENTRY_EDGE,(unsigned long)(now-run_start_ms),(long)progress,
          (unsigned int)bits,(unsigned int)parking_events);
        send_text(line);
      }
      return;
    }
    parking_feature_hits=0U;
    correction=clamp((int16_t)(git_line_error(bits)*22),100);
    drive(220+correction,220-correction);
    return;
  }
  if (state == PARKING_PAUSE) {
    stop_motors();
    if (now - state_start_ms >= PARKING_PAUSE_MS) {
      parking_pass_start_count = forward_a_counts();
      parking_pass_start_yaw_x100 = imu_yaw_x100;
      parking_line_lost_ms = 0U;
      parking_clean_hits = 0U;
      last_encoder_progress_ms = now;
      state_start_ms = now;
      state = PARKING_PASS;
      parking_event("RESUME", now, bits, parking_pass_start_count);
    }
    return;
  }
  if (state == PARKING_PASS) {
    progress = forward_a_counts();
    if (now - state_start_ms > PARKING_PASS_MAX_MS ||
        progress - parking_pass_start_count > PARKING_PASS_MAX_COUNTS ||
        yaw_change_x100(parking_pass_start_yaw_x100) >
          PARKING_PASS_MAX_YAW_X100) {
      stop_run("PARKING_PASS_LIMIT"); return;
    }
    if (bits == 0U) {
      if (parking_line_lost_ms == 0U) parking_line_lost_ms = now;
      if (now - parking_line_lost_ms > PARKING_PASS_LINE_LOST_MS) {
        stop_run("PARKING_LINE_LOST"); return;
      }
    } else parking_line_lost_ms = 0U;
    /* Keep a small amount of line correction after crossing the tape.
     * Blind equal-speed driving previously let the line slide off one side.
     */
    correction = bits ? clamp((int16_t)(git_line_error(bits) * 14), 60) : 0;
    drive(PARKING_PASS_SPEED + correction,
          PARKING_PASS_SPEED - correction);
    if ((bits & 0x18U) != 0U && active_count(bits) <= 2U &&
        progress - parking_pass_start_count >= PARKING_PASS_MIN_COUNTS) {
      if (++parking_clean_hits >= 10U) {
        state = FOLLOW;
        last_turn_complete_ms = now;
        corner_candidate = NO_TURN;
        git_corner_hits = 0U;
        parking_event("CLEARED", now, bits, progress);
      }
    } else parking_clean_hits = 0U;
    return;
  }
  if (state == FOLLOW) {
    progress = forward_a_counts();
    if (progress < -200L) { stop_run("ODOM_DIRECTION"); return; }
    if (progress > true_corner_a_counts[turns] +
                   CORNER_POSITION_WINDOW) {
      stop_run("EXPECTED_CORNER_MISSING"); return;
    }
    if (bits == 0U) {
      if (!line_lost) { line_lost = 1U; line_lost_ms = now; }
      if (now - line_lost_ms > 120U) {
        stop_run("GIT_LINE_LOST"); return;
      }
      drive(110, 110);
      return;
    }
    line_lost = 0U;
    left_black = active_count(bits & 0xF0U);
    right_black = active_count(bits & 0x0FU);
    wide_black = left_black >= 3U && right_black >= 3U;
    parking_zone = in_straight_middle(progress);
    detected = now - last_turn_complete_ms >= CORNER_COOLDOWN_MS ?
               git_detect_corner(bits) : NO_TURN;
    if (detected != NO_TURN || wide_black ||
        (parking_zone && map_garage_candidate(bits))) {
      if (near_true_corner(progress)) {
        if (detected != RIGHT_TURN) {
          stop_run("UNEXPECTED_CORNER_PATTERN"); return;
        }
        if (corner_candidate == RIGHT_TURN) git_corner_hits++;
        else { corner_candidate = RIGHT_TURN; git_corner_hits = 1U; }
        if (git_corner_hits >= 3U) {
          if (turns + 1U == CORNER_TARGET_COUNT) {
            stop_motors();
            record_corner_event("START", (uint8_t)(turns + 1U), detected, now);
            turns++;
            report(now);
            stop_run("UNEXPECTED_FULL_LAP");
            return;
          }
          record_corner_event("START", (uint8_t)(turns + 1U), detected, now);
          turn_direction = detected;
          state_start_ms = now;
          state = ENTER_CORNER;
          git_corner_hits = 0U;
          send_text("EVENT,CORNER_RIGHT\r\n");
        }
        return;
      }
      if (parking_zone) {
        if (corner_candidate == LEFT_TURN) git_corner_hits++;
        else { corner_candidate = LEFT_TURN; git_corner_hits = 1U; }
        if (git_corner_hits >= 3U) {
          if (++parking_events > PARKING_EVENT_LIMIT) {
            stop_run("PARKING_EVENT_LIMIT"); return;
          }
          stop_motors();
          state_start_ms = now;
          state = PARKING_PAUSE;
          git_corner_hits = 0U;
          parking_event("PAUSE", now, bits, progress);
        }
        return;
      }
      stop_run("UNEXPECTED_BLACK_FEATURE"); return;
    }
    corner_candidate = NO_TURN;
    git_corner_hits = 0U;
    correction = clamp((int16_t)(git_line_error(bits) * 22), 100);
    drive(220 + correction, 220 - correction);
  } else if (state == ENTER_CORNER) {
    drive(180, 180);
    if (now - state_start_ms >= 100U) {
      state_start_ms = now;
      center_hits = 0U;
      state = PIVOT;
    }
  } else if (state == PIVOT) {
    if (turn_direction == LEFT_TURN) drive(-220, 220);
    else drive(220, -220);
    if (now - state_start_ms > 1200U) {
      stop_run("GIT_TURN_TIMEOUT"); return;
    }
    if (now - state_start_ms >= 120U &&
        (bits & 0x18U) != 0U && active_count(bits) <= 2U) {
      if (++center_hits >= 3U) {
        state_start_ms = now;
        state = RECOVER;
      }
    } else center_hits = 0U;
  } else if (state == RECOVER) {
    drive(180, 180);
    if (now - state_start_ms >= 100U) {
      turns++;
      if(turns==ENTRY_REQUIRED_CORNERS)parking_corner_progress=forward_a_counts();
      if (turns >= CORNER_TARGET_COUNT) stop_motors();
      record_corner_event("DONE", turns, turn_direction, now);
      if (turns >= CORNER_TARGET_COUNT) {
        report(now);
        stop_run("FOUR_CORNERS_COMPLETE");
      } else {
        state = FOLLOW;
        last_turn_complete_ms = now;
        corner_candidate = NO_TURN;
      }
    }
  }
}
