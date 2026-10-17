#include "mv_client.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <limits.h>

static uint32_t age(uint32_t now, uint32_t then) { return now - then; }
static uint8_t equal(const char *a, const char *b) { return strcmp(a, b) == 0; }

static uint8_t decimal(const char *s, uint32_t max, uint32_t *out)
{
  uint32_t value = 0U;
  if (!*s) return 0U;
  for (; *s; ++s) {
    if (*s < '0' || *s > '9') return 0U;
    uint32_t digit = (uint32_t)(*s - '0');
    if (digit > max || value > (max - digit) / 10U) return 0U;
    value = value * 10U + digit;
  }
  *out = value;
  return 1U;
}

static int hex_digit(char ch)
{
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

static uint8_t session(const char *s, uint32_t *out)
{
  uint32_t value = 0U;
  if (strlen(s) != 8U) return 0U;
  for (size_t i = 0U; i < 8U; ++i) {
    int digit = hex_digit(s[i]);
    if (digit < 0) return 0U;
    value = (value << 4) | (uint32_t)digit;
  }
  if (!value) return 0U;
  *out = value;
  return 1U;
}

static void packet(MvClient *c, const char *format, ...)
{
  char wire[MV_MAX_FRAME + 1U];
  uint8_t checksum = 0U;
  va_list args;
  va_start(args, format);
  int n = vsnprintf(wire + 1, sizeof(wire) - 1U, format, args);
  va_end(args);
  if (n <= 0 || n > (int)MV_MAX_FRAME - 6) { c->io_fault = 1U; return; }
  wire[0] = '$';
  for (int i = 1; i <= n; ++i) checksum ^= (uint8_t)wire[i];
  (void)snprintf(wire + n + 1, sizeof(wire) - (size_t)n - 1U,
                 "*%02X\r\n", (unsigned int)checksum);
  if (!c->send(wire, (uint16_t)(n + 6))) c->io_fault = 1U;
}

static void event(MvClient *c, MvEventKind kind, const char *reason)
{
  memset(&c->event, 0, sizeof(c->event));
  c->event.kind = kind;
  c->event.sid = c->sid;
  c->event.seq = c->seq;
  c->event.task = c->task;
  (void)snprintf(c->event.reason, sizeof(c->event.reason), "%s", reason);
  c->event_pending = 1U;
}

static void sync_again(MvClient *c, uint32_t now, uint8_t lost)
{
  c->expected = c->queued = c->saved = c->have_rx = 0U;
  c->phase = MV_HELLO;
  c->attempts = 0U;
  c->now = now;
  c->last_tx = now - 1000U;
  if (lost) event(c, MV_LOST, "SESSION_CHANGED");
}

static void send_set(MvClient *c, uint32_t now)
{
  packet(c, "MV,1,SET,%08lX,%lu,%u,%lu", (unsigned long)c->sid,
         (unsigned long)c->seq, (unsigned int)c->task, (unsigned long)c->timeout_ms);
  c->last_tx = now;
  ++c->attempts;
}

static uint8_t new_set(MvClient *c, uint8_t task, uint32_t now)
{
  if (c->last_seq == UINT32_MAX) {
    c->expected = c->queued = 0U;
    c->phase = MV_OFFLINE;
    event(c, MV_FAILED, "SEQ_EXHAUSTED");
    return 0U;
  }
  c->seq = ++c->last_seq;
  c->task = task;
  c->timeout_ms = (task == 3U || task == 4U) ? MV_IDENTIFY_MS : 0U;
  c->started = now;
  c->attempts = 0U;
  /* Install expectation BEFORE enqueueing SET; ACK may arrive after RESULT. */
  c->expected = 1U;
  c->phase = (task == 3U || task == 4U) ? MV_WAIT : MV_CLEAR;
  send_set(c, now);
  return 1U;
}

void mv_init(MvClient *c, MvSend send, uint32_t now)
{
  memset(c, 0, sizeof(*c));
  c->send = send;
  sync_again(c, now, 0U);
}

uint8_t mv_link_ok(const MvClient *c, uint32_t now)
{
  return c->have_rx && !c->io_fault && c->phase != MV_HELLO &&
         c->phase != MV_OFFLINE && age(now, c->last_rx) <= 5000U;
}

static uint8_t request_task(MvClient *c, uint8_t task, uint32_t now)
{
  if (c->phase == MV_WAIT || c->queued || c->event_pending) return 0U;
  if (c->phase == MV_OFFLINE) sync_again(c, now, 0U);
  if (c->io_fault) return 0U;
  c->queued = task;
  if (c->phase == MV_READY) {
    c->queued = 0U;
    return new_set(c, task, now);
  }
  return 1U;
}

uint8_t mv_request_roman(MvClient *c, uint32_t now) { return request_task(c, 3U, now); }
uint8_t mv_request_shape(MvClient *c, uint32_t now) { return request_task(c, 4U, now); }

void mv_cancel(MvClient *c, uint32_t now)
{
  c->expected = c->queued = c->event_pending = 0U;
  if (c->sid && c->phase != MV_HELLO && c->phase != MV_OFFLINE)
    (void)new_set(c, 0U, now);
}

uint8_t mv_take_event(MvClient *c, MvEvent *out)
{
  if (!c->event_pending) return 0U;
  *out = c->event;
  c->event_pending = 0U;
  return 1U;
}

static void touch(MvClient *c, uint32_t now)
{
  c->last_rx = now;
  c->have_rx = 1U;
}

static uint8_t valid_state(const char *s)
{
  return equal(s, "IDLE") || equal(s, "ROMAN") || equal(s, "SHAPE");
}

static void ack_terminal(MvClient *c, uint32_t sid, uint32_t seq)
{
  packet(c, "MV,1,ACK,%08lX,%lu", (unsigned long)sid, (unsigned long)seq);
}

static uint8_t same_terminal(const MvEvent *a, const MvEvent *b)
{
  return a->kind == b->kind && a->target == b->target &&
         a->number == b->number && a->score == b->score && a->task == b->task &&
         equal(a->color, b->color) && equal(a->shape, b->shape) &&
         equal(a->action, b->action) && equal(a->reason, b->reason);
}

static void receive_terminal(MvClient *c, const MvEvent *terminal, uint32_t now)
{
  if (terminal->sid != c->sid) return;
  if (c->saved && terminal->seq == c->terminal.seq) {
    if (same_terminal(&c->terminal, terminal)) {
      touch(c, now);
      ack_terminal(c, terminal->sid, terminal->seq);
    } else ++c->conflicts;
    return;
  }
  if (!c->expected || c->phase != MV_WAIT || terminal->task != c->task ||
      terminal->seq != c->seq) return;
  if (age(now, c->started) >= c->timeout_ms + 3000U) {
    c->expected = c->queued = 0U;
    event(c, MV_FAILED, "LOCAL_TIMEOUT");
    (void)new_set(c, 0U, now);
    return;
  }
  /* Save first; enqueue short ACK; offer exactly one event to the car. */
  c->terminal = *terminal;
  c->saved = 1U;
  c->expected = 0U;
  c->phase = MV_READY;
  c->event = *terminal;
  c->event_pending = 1U;
  touch(c, now);
  ack_terminal(c, terminal->sid, terminal->seq);
}

static void body(MvClient *c, char *raw, uint32_t now)
{
  char *f[12];
  unsigned int count = 1U;
  uint32_t sid, seq, a, b, d;
  f[0] = raw;
  for (char *p = raw; *p; ++p) {
    if (*p == ',') {
      if (count == 12U) { ++c->bad_frames; return; }
      *p = '\0'; f[count++] = p + 1;
    } else if (*p < 32 || *p > 126 || *p == '*' || *p == '$') {
      ++c->bad_frames; return;
    }
  }
  for (unsigned int i = 0U; i < count; ++i) if (!*f[i]) { ++c->bad_frames; return; }
  if (count < 5U || !equal(f[0], "MV") || !equal(f[1], "1") ||
      !session(f[3], &sid) || !decimal(f[4], UINT32_MAX, &seq)) {
    ++c->bad_frames; return;
  }
  if ((equal(f[2], "READY") || equal(f[2], "STATE")) && count == 9U) {
    if (seq || !valid_state(f[5]) || !decimal(f[6], UINT32_MAX, &a) ||
        !decimal(f[7], 255U, &b) || !decimal(f[8], 1U, &d)) { ++c->bad_frames; return; }
    if (c->phase == MV_HELLO) {
      if (!equal(f[2], "READY")) return;
      c->sid = sid;
      c->last_seq = a;
      touch(c, now);
      c->last_ping = now;
      (void)new_set(c, 0U, now);
    } else if (sid != c->sid) {
      sync_again(c, now, 1U); /* A new STATE is a clue, not permission to drive. */
    } else {
      touch(c, now); /* Same-session READY never clears a valid transaction. */
    }
    return;
  }
  if (sid != c->sid || c->phase == MV_HELLO || c->phase == MV_OFFLINE) return;
  if (equal(f[2], "PONG") && count == 5U && seq == 0U) { touch(c, now); return; }
  if (equal(f[2], "ACK") && count == 7U && c->expected && seq == c->seq) {
    if (!decimal(f[5], 255U, &a) || a != c->task ||
        !(valid_state(f[6]) || equal(f[6], "DONE"))) { ++c->bad_frames; return; }
    if ((c->task == 0U && !equal(f[6], "IDLE")) ||
        (c->task == 3U && !(equal(f[6], "ROMAN") || equal(f[6], "DONE"))) ||
        (c->task == 4U && !(equal(f[6], "SHAPE") || equal(f[6], "DONE")))) {
      ++c->bad_frames; return;
    }
    touch(c, now);
    if (c->phase == MV_CLEAR) { c->expected = 0U; c->phase = MV_READY; }
    else c->attempts = 3U; /* Accepted, but still wait for RESULT or FAIL. */
    return;
  }
  if (equal(f[2], "RESULT") && count == 11U) {
    if (equal(f[5], "4")) {
      if (!equal(f[6], "SHAPE") ||
          !(equal(f[8], "CIRCLE") || equal(f[8], "TRIANGLE") || equal(f[8], "SQUARE")) ||
          !((equal(f[7], "RED") && equal(f[9], "REVERSE")) ||
            (equal(f[7], "YELLOW") && equal(f[9], "SIDE"))) ||
          !decimal(f[10], 100U, &b)) { ++c->bad_frames; return; }
      MvEvent result = {0};
      result.kind = MV_SHAPE; result.task = 4U;
      result.sid = sid; result.seq = seq; result.score = (uint8_t)b;
      (void)snprintf(result.color, sizeof(result.color), "%s", f[7]);
      (void)snprintf(result.shape, sizeof(result.shape), "%s", f[8]);
      (void)snprintf(result.action, sizeof(result.action), "%s", f[9]);
      receive_terminal(c, &result, now);
      return;
    }
    static const char *symbols[] = {"I", "III", "V", "VII"};
    static const uint8_t numbers[] = {1U, 3U, 5U, 7U};
    if (!equal(f[5], "3") || !equal(f[6], "ROMAN") ||
        !decimal(f[8], 7U, &a) || !decimal(f[10], 100U, &b) ||
        strlen(f[9]) != 1U) { ++c->bad_frames; return; }
    for (unsigned int i = 0U; i < 4U; ++i) {
      if (equal(f[7], symbols[i]) && a == numbers[i] && f[9][0] == (char)('A' + i)) {
        MvEvent result = {0};
        result.kind = MV_ROMAN; result.task = 3U; result.sid = sid; result.seq = seq;
        result.target = f[9][0]; result.number = (uint8_t)a; result.score = (uint8_t)b;
        receive_terminal(c, &result, now);
        return;
      }
    }
    ++c->bad_frames; return;
  }
  if (equal(f[2], "FAIL") && count == 7U && (equal(f[5], "3") || equal(f[5], "4"))) {
    if (!(equal(f[6], "TIMEOUT") || equal(f[6], "LINK") ||
          equal(f[6], "CAMERA") || equal(f[6], "VISION"))) { ++c->bad_frames; return; }
    MvEvent failure = {0};
    failure.kind = MV_FAILED; failure.sid = sid; failure.seq = seq;
    failure.task = (uint8_t)(f[5][0] - '0');
    (void)snprintf(failure.reason, sizeof(failure.reason), "%s", f[6]);
    receive_terminal(c, &failure, now);
    return;
  }
  if (equal(f[2], "ERROR") && count == 6U && c->expected && seq == c->seq) {
    if (!(equal(f[5], "SESSION") || equal(f[5], "STALE") || equal(f[5], "CONFLICT") ||
          equal(f[5], "ARGS") || equal(f[5], "COMMAND"))) { ++c->bad_frames; return; }
    touch(c, now);
    c->expected = c->queued = 0U;
    c->phase = MV_OFFLINE;
    event(c, MV_FAILED, f[5]); /* No ERROR echo, no unbounded new-SEQ retry. */
  }
}

void mv_reset_rx(MvClient *c) { c->rx_length = 0U; ++c->bad_frames; }

void mv_feed(MvClient *c, uint8_t byte, uint32_t now)
{
  c->now = now;
  if (byte == '$') { c->rx[0] = '$'; c->rx_length = 1U; return; }
  if (!c->rx_length) return;
  if (c->rx_length >= MV_MAX_FRAME) { mv_reset_rx(c); return; }
  c->rx[c->rx_length++] = (char)byte;
  if (byte != '\n') return;
  uint16_t n = c->rx_length;
  c->rx_length = 0U;
  if (n < 7U || c->rx[n - 2U] != '\r' || c->rx[n - 5U] != '*') {
    ++c->bad_frames; return;
  }
  int high = hex_digit(c->rx[n - 4U]), low = hex_digit(c->rx[n - 3U]);
  uint8_t sum = 0U;
  for (uint16_t i = 1U; i < n - 5U; ++i) sum ^= (uint8_t)c->rx[i];
  if (high < 0 || low < 0 || sum != (uint8_t)((high << 4) | low)) {
    ++c->bad_frames; return;
  }
  c->rx[n - 5U] = '\0';
  body(c, c->rx + 1, now);
}

void mv_poll(MvClient *c, uint32_t now)
{
  c->now = now;
  if (c->io_fault) {
    c->expected = c->queued = 0U;
    if (c->phase != MV_OFFLINE) event(c, MV_LOST, "TX_QUEUE_FULL");
    c->phase = MV_OFFLINE;
    return;
  }
  if (c->phase == MV_HELLO) {
    if (age(now, c->last_tx) >= 1000U) {
      if (c->attempts >= 3U) {
        c->phase = MV_OFFLINE; c->queued = 0U; event(c, MV_LOST, "HELLO_TIMEOUT");
      } else {
        packet(c, "MV,1,HELLO,00000000,0");
        c->last_tx = now; ++c->attempts;
      }
    }
    return;
  }
  if (c->phase == MV_OFFLINE) return;
  if (!mv_link_ok(c, now)) {
    c->expected = c->queued = 0U; c->phase = MV_OFFLINE;
    event(c, MV_LOST, "LINK_TIMEOUT");
    return;
  }
  if (age(now, c->last_ping) >= 1000U) {
    packet(c, "MV,1,PING,%08lX,0", (unsigned long)c->sid);
    c->last_ping = now;
  }
  if (c->phase == MV_WAIT && age(now, c->started) >= c->timeout_ms + 3000U) {
    c->expected = 0U;
    event(c, MV_FAILED, "LOCAL_TIMEOUT");
    c->queued = 0U;
    (void)new_set(c, 0U, now); /* Invalidate first, then cancel remotely. */
    return;
  }
  if (c->expected && age(now, c->last_tx) >= 1000U) {
    if (c->attempts < 3U) send_set(c, now);
    else if (c->phase == MV_CLEAR) {
      c->expected = c->queued = 0U; c->phase = MV_OFFLINE;
      event(c, MV_FAILED, "SET0_ACK_TIMEOUT");
    }
  }
  if (c->phase == MV_READY && c->queued) {
    uint8_t task = c->queued;
    c->queued = 0U;
    (void)new_set(c, task, now);
  }
}
