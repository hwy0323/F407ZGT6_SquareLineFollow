#ifndef SEARCH_CONFIG_H
#define SEARCH_CONFIG_H

/* Positive = window is inside/right of the track centre, in millimetres.
 * User-approved estimate: half the 147 mm chassis width, rounded to 74 mm.
 * This is not a measured optical/mechanical calibration. */
#ifndef LASER_RIGHT_OFFSET_MM
#define LASER_RIGHT_OFFSET_MM 74
#endif
#ifndef LASER_OFFSET_CONFIRMED
#define LASER_OFFSET_CONFIRMED 1
#endif
#define SIDE_WALL_DEPTH_MM 200
#define REVERSE_WALL_DEPTH_MM 300
#define DEPTH_TOLERANCE_MM 25
#define RANGE_SPREAD_MAX_MM 25
#define RANGE_SAMPLE_COUNT 5U
#define PROBE_COUNTS 287L /* round(50 mm * 5.73 counts/mm) */
#define STEP_GAP_MS 150U
#define MEASURE_TIMEOUT_MS 2500U
#define SEARCH_MAX_MS 45000U
#define JOB_MAX_MS 90000U
#define CORNER_WINDOW_COUNTS 900L
#define SEARCH_START_FRAME "@TASK5:RUN"
#define ENTRY_NAME "EXT2_LASER_SEARCH_UNTESTED"

#endif
