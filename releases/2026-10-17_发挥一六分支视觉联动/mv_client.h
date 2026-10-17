#ifndef MV_CLIENT_H
#define MV_CLIENT_H
#include <stddef.h>
#include <stdint.h>

#define MV_MAX_FRAME 192U
#define MV_IDENTIFY_MS 30000U
typedef uint8_t (*MvSend)(const char *data, uint16_t length);
typedef enum { MV_HELLO, MV_CLEAR, MV_READY, MV_WAIT, MV_OFFLINE } MvPhase;
typedef enum { MV_NO_EVENT, MV_ROMAN, MV_FAILED, MV_LOST, MV_SHAPE } MvEventKind;
typedef struct {
  MvEventKind kind;
  uint32_t sid, seq;
  char target;
  uint8_t number, score;
  char reason[24];
  uint8_t task;
  char color[7], shape[9], action[8];
} MvEvent;
typedef struct {
  MvSend send;
  MvPhase phase;
  uint32_t sid, last_seq, seq, timeout_ms;
  uint32_t now, last_rx, last_ping, last_tx, started;
  uint32_t bad_frames, conflicts;
  uint8_t attempts, expected, task, queued, event_pending;
  uint8_t saved, have_rx, io_fault;
  uint16_t rx_length;
  char rx[MV_MAX_FRAME + 1U];
  MvEvent event, terminal;
} MvClient;

void mv_init(MvClient *c, MvSend send, uint32_t now);
void mv_feed(MvClient *c, uint8_t byte, uint32_t now);
void mv_poll(MvClient *c, uint32_t now);
uint8_t mv_request_roman(MvClient *c, uint32_t now);
uint8_t mv_request_shape(MvClient *c, uint32_t now);
void mv_cancel(MvClient *c, uint32_t now);
uint8_t mv_take_event(MvClient *c, MvEvent *event);
uint8_t mv_link_ok(const MvClient *c, uint32_t now);
void mv_reset_rx(MvClient *c);
#endif
