/* New orchestration only; navigation and precise motion keep their sources. */
static uint8_t combined_start_feed(uint8_t byte)
{
  static char command[16];
  static uint8_t used, collecting;
  if (byte == '@') { used = 0U; collecting = 1U; }
  if (!collecting || byte == '\r') return 0U;
  if (byte == '\n') {
    command[used] = 0; collecting = 0U;
    return strcmp(command, "@SIDE:RUN") == 0;
  }
  if (byte < 32U || byte > 126U || used >= sizeof(command) - 1U) {
    used = collecting = 0U; return 0U;
  }
  command[used++] = (char)byte;
  return 0U;
}

static void combined_abort(const char *reason)
{
  char line[100];
  if (pivot_active) reference_finish(reason);
  if (state != STOPPED) stop_run(reason);
  auto_phase = 4U;
  reference_halted = 1U;
  stop_motors();
  (void)snprintf(line, sizeof(line), "EVENT,SIDE_REFERENCE_ABORT,%s\r\n", reason);
  send_text(line);
}

static void combined_tick(uint32_t now)
{
  if (auto_phase == 2U && now - auto_pause_start_ms >= 1200U) {
    /* Match the fully stopped pose used for the accepted single-step trial. */
    auto_phase = 3U;
    send_text("EVENT,SIDE_REFERENCE_ENTRY_BEGIN,merged_reverse_1347_counts\r\n");
    reference_start(now);
    if (!pivot_active) combined_abort("FIRST_STEP_REFUSED");
    return;
  }
  if (auto_phase != 3U) return;
  if (reference_halted) { combined_abort("REFERENCE_STEP_FAILED"); return; }
  if (pivot_active) return;
  if (reference_next == 5U) {
    stop_motors(); auto_phase = 4U;
    send_text("EVENT,SIDE_REFERENCE_DONE,entry_only,motors_off\r\n");
    return;
  }
  if (now - pivot_last_stop_ms >= 150U) {
    reference_start(now);
    if (!pivot_active) combined_abort("NEXT_STEP_REFUSED");
  }
}
