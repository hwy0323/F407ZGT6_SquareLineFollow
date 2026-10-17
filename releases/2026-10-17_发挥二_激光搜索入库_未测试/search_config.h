#ifndef SEARCH_CONFIG_H
#define SEARCH_CONFIG_H

/* 用户实测的原始激光读数，单位 mm；不再叠加半车宽偏移。
 * 159 mm 为侧停，315 mm 为倒库；237 mm 恰好是二者中点。
 * 中位数 <= 237 mm 判侧停，> 237 mm 判倒库。 */
#define LASER_SIDE_REFERENCE_MM 159U
#define LASER_REVERSE_REFERENCE_MM 315U
#define LASER_TYPE_THRESHOLD_MM 237U
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
