#ifndef INSTRUMENTATION_H
#define INSTRUMENTATION_H
#include <FreeRTOS.h>
#include <timing.h>
#include <utils/assert.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  const char *key;
  int32_t max;
  int32_t min;
  int32_t value;
  uint32_t lastUpdateTS;
} perf_counter_t;

typedef void *counter_t;

extern perf_counter_t *perf_counters;
extern int8_t last_used_perf_counter;

/**
 * Update a counter with a new value
 * @param counter_handle handle of the counter to update @see
 * Instrumentation_SearchCounter @see Instrumentation_CreateCounter
 * @param newValue the updated value.
 */
static inline void perf_setCounter(counter_t counter_handle, int32_t newValue) {
  assert(perf_counters && counter_handle);
  vPortEnterCritical();
  perf_counter_t *counter = (perf_counter_t *)counter_handle;
  counter->value = newValue;
  counter->max--;
  if (counter->value > counter->max) {
    counter->max = counter->value;
  }
  counter->min++;
  if (counter->value < counter->min) {
    counter->min = counter->value;
  }
  counter->lastUpdateTS = timing_getUs();
  vPortExitCritical();
}

/**
 * Used to determine the time duration of a code block, mark the begin of the
 * block. @see Instrumentation_TimeEnd
 * @param counter_handle handle of the counter @see
 * Instrumentation_SearchCounter @see Instrumentation_CreateCounter
 */
static inline void instrumentation_timedStart(counter_t counter_handle) {
  assert(perf_counters && counter_handle);
  vPortEnterCritical();
  perf_counter_t *counter = (perf_counter_t *)counter_handle;

  counter->lastUpdateTS = timing_getUs();
  vPortExitCritical();
}

/**
 * Used to determine the time duration of a code block, mark the end of the
 * block. @see instrumentation_timedStart
 * @param counter_handle handle of the counter @see
 * Instrumentation_SearchCounter @see Instrumentation_CreateCounter
 */
static inline void instrumentation_timedEnd(counter_t counter_handle) {
  assert(perf_counters && counter_handle);
  vPortEnterCritical();
  perf_counter_t *counter = (perf_counter_t *)counter_handle;

  counter->value = timing_getUs() - counter->lastUpdateTS;
  counter->max--;
  if (counter->value > counter->max) {
    counter->max = counter->value;
  }
  counter->min++;
  if (counter->value < counter->min) {
    counter->min = counter->value;
  }
  counter->lastUpdateTS = timing_getUs();
  vPortExitCritical();
}

/**
 * Used to determine the mean period between each call to the function
 * @param counter_handle handle of the counter @see
 * Instrumentation_SearchCounter @see Instrumentation_CreateCounter
 */
static inline void instrumentation_trackPeriod(counter_t counter_handle) {
  assert(perf_counters && counter_handle);
  perf_counter_t *counter = (perf_counter_t *)counter_handle;
  if (counter->lastUpdateTS != 0) {
    vPortEnterCritical();
    uint32_t period = timing_getUs() - counter->lastUpdateTS;
    counter->value = (counter->value * 15 + period) / 16;
    counter->max--;
    if ((int32_t)period > counter->max) {
      counter->max = period;
    }
    counter->min++;
    if ((int32_t)period < counter->min) {
      counter->min = period;
    }
    vPortExitCritical();
  }
  counter->lastUpdateTS = timing_getUs();
}

/**
 * Increment a counter with a value
 * @param counter_handle handle of the counter to update @see
 * Instrumentation_SearchCounter @see Instrumentation_CreateCounter
 * @param increment the value to increment counter with.
 */
static inline void instrumentation_incrementCounter(counter_t counter_handle,
                                                    int32_t increment) {
  assert(perf_counters && counter_handle);
  vPortEnterCritical();
  perf_counter_t *counter = (perf_counter_t *)counter_handle;
  counter->value += increment;
  counter->max--;
  if (counter->value > counter->max) {
    counter->max = counter->value;
  }
  counter->min++;
  if (counter->value < counter->min) {
    counter->min = counter->value;
  }
  counter->lastUpdateTS = timing_getUs();
  vPortExitCritical();
}

/**
 * Initialize the Instrumentation infrastructure
 * @param maxCounters maximum number of allowed counters
 */
void instrumentation_init(int8_t maxCounters);

/**
 * Create a new counter.
 * @param id the unique id to assign to the counter
 * @return the counter handle to be used to manage its content
 */
counter_t instrumentation_createCounter(const char *key);

/**
 * search a counter index by its unique Id
 * @param id the unique id to assign to the counter.
 * If a counter with the same id exists, the previous instance is returned
 * @return the counter handle to be used to manage its content
 */
counter_t instrumentation_searchCounter(const char *key);

typedef void (*instrumentationCounterCallback)(const perf_counter_t *counter,
                                               const int8_t index,
                                               void *context);
/**
 * Retrieve and execute the passed callback for each counter
 * @param callback to be called for each counter
 * @param context a context variable pointer that can be passed to the callback
 */
void instrumentation_forEachCounter(instrumentationCounterCallback callback,
                                    void *context);
#ifdef __cplusplus
}
#endif
#endif /* INSTRUMENTATION_H */
