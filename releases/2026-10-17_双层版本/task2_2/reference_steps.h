#ifndef REFERENCE_STEPS_H
#define REFERENCE_STEPS_H
#include <stdint.h>

typedef struct { char direction; int32_t target; const char *command; } ReferenceStep;
/* L/R = 0.01 degree; F/B = encoder A/B/C median counts, NOT millimetres. */
extern const ReferenceStep reference_steps[5];
typedef struct { char line[24]; uint8_t used, collecting; uint16_t anchor; } ReferenceParser;
/* Feed returns 1 for the expected move, 2 for a first-step baseline, 0 otherwise. */
uint8_t reference_feed(ReferenceParser *parser, uint8_t byte, uint8_t expected);
int32_t reference_yaw_delta(uint16_t start, uint16_t current);
int32_t reference_encoder_progress(int16_t a, int16_t b, int16_t c, int8_t direction);
#endif
