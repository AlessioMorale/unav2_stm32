#pragma once
/* each entry contains:

*/
#define COUNTERS_KEY_TABLE(ENTRY)                                              \
  ENTRY(action_latency, "AcLate", "General", "Action latency", "uS", 1)        \
  ENTRY(mc_loop_time, "McLoop", "Motor Controller",                            \
        "Motor controller loop time", "uS", 1)

#define COUNTERS_DECLARE(x, y, ...) extern const char *counters_##x##_key;
#define COUNTERS_DEFINE(x, y, ...) const char *counters_##x##_key = y;
#define COUNTERS_COUNT_EXPAND(...) (1) +
#define COUNTERS_COUNT COUNTERS_KEY_TABLE(COUNTERS_COUNT_EXPAND) 0

COUNTERS_KEY_TABLE(COUNTERS_DECLARE)
