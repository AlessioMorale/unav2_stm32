#include <instrumentation/instrumentation.h>

perf_counter_t *perf_counters = NULL;
int8_t instrumentation_max_counters = -1;
int8_t last_used_perf_counter = -1;

void instrumentation_init(int8_t maxCounters) {
  assert(maxCounters >= 0);
  if (maxCounters > 0) {
    perf_counters =
        (perf_counter_t *)pvPortMalloc(sizeof(perf_counter_t) * maxCounters);
    assert(perf_counters);
    memset(perf_counters, 0, sizeof(perf_counter_t) * maxCounters);
    instrumentation_max_counters = maxCounters;
  } else {
    perf_counters = NULL;
    instrumentation_max_counters = -1;
  }
}

counter_t instrumentation_createCounter(uint32_t id) {
  assert(perf_counters &&
         (instrumentation_max_counters > last_used_perf_counter));

  counter_t counter_handle = instrumentation_searchCounter(id);
  if (!counter_handle) {
    perf_counter_t *newcounter = &perf_counters[++last_used_perf_counter];
    newcounter->id = id;
    newcounter->max = INT32_MIN + 1;
    newcounter->min = INT32_MAX - 1;
    counter_handle = (counter_t)newcounter;
  }
  return counter_handle;
}

counter_t instrumentation_searchCounter(uint32_t id) {
  assert(perf_counters);
  uint8_t i = 0;
  while (i < last_used_perf_counter && perf_counters[i].id != id) {
    i++;
  }
  if (perf_counters[i].id != id) {
    return NULL;
  }
  return (counter_t)&perf_counters[i];
}

void instrumentation_forEachCounter(instrumentationCounterCallback callback,
                                    void *context) {
  assert(perf_counters);
  for (int8_t index = 0; index < last_used_perf_counter + 1; index++) {
    const perf_counter_t *counter = &perf_counters[index];
    callback(counter, index, context);
  }
}
