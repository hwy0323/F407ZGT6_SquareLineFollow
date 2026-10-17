#ifndef TASK4_CONFIG_H
#define TASK4_CONFIG_H
#include <stdint.h>
/* Midpoints from the measured pairs, not a reverse/side/reverse layout rule.
 * A-wheel cumulative counts include turning and must not be read as mm. */
static const int32_t garage_midpoint_counts[3]={8427L,14404L,20527L};
#define TASK4_LINE_WINDOW 500L
#define TASK4_COUNTS_PER_100_MM 573L
/* Along-track spans inferred from the current recorded map: 300 / 400 mm.
 * Separate from the two garage depths. Recalibrate if tape orientation changes. */
#define TASK4_REVERSE_ALONG_MM 300L
#define TASK4_SIDE_ALONG_MM 400L
/* Branch order: BC reverse, BC side, CD reverse, CD side, DA reverse, DA side.
 * Per-branch offsets allow later tuning without moving every other garage. */
static const int32_t first_line_trim[6]={0,0,0,0,0,0};
/* Counts for F/B, 0.01 degree for L/R. No automatic mirror/swap on another edge. */
static const int32_t motion_trim[6][5]={{0},{0},{0},{0},{0},{0}};
#endif
