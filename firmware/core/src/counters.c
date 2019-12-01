#define INSTRUMENT_MODULE
#include <counters.h>

COUNTERS_KEY_TABLE(COUNTERS_DEFINE_KEY)

COUNTERS_KEY_TABLE(COUNTERS_DEFINE_COUNTER)

void initCounters() {
  instrumentation_init(COUNTERS_COUNT);
  COUNTERS_KEY_TABLE(COUNTERS_INIT_COUNTER)
}