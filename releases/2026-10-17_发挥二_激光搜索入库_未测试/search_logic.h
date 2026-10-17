#ifndef SEARCH_LOGIC_H
#define SEARCH_LOGIC_H
#include <stdint.h>
#include "search_config.h"
typedef enum { GARAGE_UNKNOWN, GARAGE_SIDE, GARAGE_REVERSE } GarageType;
typedef enum { FEATURE_LINE, FEATURE_CORNER, FEATURE_GARAGE,
               FEATURE_AMBIGUOUS, FEATURE_END } TrackFeature;
extern const int32_t search_corner_counts[4];
TrackFeature search_feature(uint8_t bits, uint8_t corners, int32_t progress);
GarageType search_classify(const uint16_t samples[RANGE_SAMPLE_COUNT],
                           uint16_t *median_mm);
#endif
