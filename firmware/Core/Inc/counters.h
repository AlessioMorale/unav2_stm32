#pragma once
#define INSTRUMENT_MODULE
#include "instrumentation/instrumentation_helper.h"

/* each entry contains:
ENTRY(variable_name, "counter_key", "Counter category","Counter
description","UOM", scale)
*/
#define COUNTERS_KEY_TABLE(ENTRY)                                              \
  ENTRY(action_latency, "AcLate", "System", "Action latency", "uS", 1)         \
  ENTRY(mc_loop_time, "McLoop", "Motor Controller",                            \
        "Motor controller loop time", "uS", 1)                                 \
  ENTRY(sys_free_msg, "SysFrMsg", "System", "Free messages slots", "", 1)

#define COUNTERS_DECLARE_KEY(x, y, ...) extern const char *counters_##x##_key;
#define COUNTERS_DEFINE_KEY(x, y, ...) const char *counters_##x##_key = y;
#define COUNTERS_DECLARE_COUNTER(x, y, ...) PERF_DECLARE_COUNTER(perf_##x);
#define COUNTERS_DEFINE_COUNTER(x, y, ...) PERF_DEFINE_COUNTER(perf_##x);
#define COUNTERS_INIT_COUNTER(x, y, ...)                                       \
  PERF_INIT_COUNTER(perf_##x, counters_##x##_key);
#define COUNTERS_COUNT_EXPAND(...) (1) +
#define COUNTERS_COUNT COUNTERS_KEY_TABLE(COUNTERS_COUNT_EXPAND) 0

COUNTERS_KEY_TABLE(COUNTERS_DECLARE_KEY)

COUNTERS_KEY_TABLE(COUNTERS_DECLARE_COUNTER)

#ifdef __cplusplus
extern "C" {
#endif
void initCounters();
#ifdef __cplusplus
}
#endif