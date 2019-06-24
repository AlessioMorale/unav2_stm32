/**
 * \par
 * This is a collections of helper macros that ease adding instrumentation
 * support. \par Step by step guide:
 *
 * Define INSTRUMENT_MODULE before including this file to enable instrumentation
 * for a module
 *
 * <pre>#define INSTRUMENT_MODULE
 * #include <instrumentation_helper.h></pre>
 *
 * Declare the variables used to hold counter handlers.
 * Place the following code along all module variables declaration.
 * <pre>PERF_DEFINE_COUNTER(counterUpd);
 * PERF_DEFINE_COUNTER(counterAccelSamples);
 * PERF_DEFINE_COUNTER(counterPeriod);
 * PERF_DEFINE_COUNTER(counterAtt);</pre>
 *
 * Counters needs to be initialized before they are used.
 * The following code needs to be added to a function called at module
 * initialization. the second <pre>PERF_TIMED_SECTION_START(counterAtt);
 * updateAttitude(&accelState, &gyros);
 * PERF_TIMED_SECTION_END(counterAtt);</pre>
 * PERF_TIMED_SECTION_[START!STOP] marks the beginning and the end of the code
 * to monitor
 *
 * Measure the mean of the period a certain point is reached:
 * <pre>PERF_MEASURE_PERIOD(counterPeriod);</pre>
 * Note that the value stored in the counter is a long running mean while max
 * and min are single point values
 *
 * Track an user defined int32_t value: parameter is a unique counter Id.
 * A good pracice is to use the upper half word as module id and lower as
 * counter id Optionally three strings containing Module Name, Counter
 * description and unit of measure can be passed. Those strings will be used in
 * future to automatically extract the list of counters from code to managed by
 * GCS or some other custom tool.
 *
 * PERF_INIT_COUNTER(counterVariable, id, "MODULE NAME","COUNTER
 * DESCRIPTION","UNIT OF MEASURE");
 *
 * <pre>PERF_INIT_COUNTER(counterUpd, 0xA7710001, "ATTITUDE", "Sensor update
 * execution time", "us"); PERF_INIT_COUNTER(counterAtt, 0xA7710002, "ATTITUDE",
 * "Attitude estimation execution time", "us"); PERF_INIT_COUNTER(counterPeriod,
 * 0xA7710003, "ATTITUDE", "Sensor update period", "us");
 * PERF_INIT_COUNTER(counterAccelSamples, 0xA7710004, "ATTITUDE", "Samples for
 * each sensor cycle", "count");</pre>
 *
 * At this point you can start using the counters as in the following samples
 *
 * Track the time spent on a certain function:
 * <pre>PERF_TIMED_SECTION_START(counterAtt);
 * updateAttitude(&accelState, &gyros);
 * PERF_TIMED_SECTION_END(counterAtt);</pre>
 * PERF_TIMED_SECTION_[START!STOP] marks the beginning and the end of the code
 * to monitor
 *
 * Measure the mean of the period a certain point is reached:
 * <pre>PERF_MEASURE_PERIOD(counterPeriod);</pre>
 * Note that the value stored in the counter is a long running mean while max
 * and min are single point values
 *
 * Track an user defined int32_t value:
 * <pre>PERF_TRACK_VALUE(counterAccelSamples, i);</pre>
 * the counter is then updated with the value of i.
 *
 * \par
 */

#ifndef INSTRUMENTATION_HELPER_H
#define INSTRUMENTATION_HELPER_H

#ifdef INSTRUMENT_MODULE

#include <instrumentation/instrumentation.h>
/**
 * include the following macro together with modules variable declaration
 */
#define PERF_DEFINE_COUNTER(x) counter_t x
#define PERF_DECLARE_COUNTER(x) extern PERF_DEFINE_COUNTER(x)
#define PERF_USE_EXTERNAL_COUNTER(x) PERF_DECLARE_COUNTER(x)
/**
 * initialize a counter and assing an id
 */
#define PERF_INIT_COUNTER(x, id) x = instrumentation_createCounter(id)

/**
 * those are the monitoring macros
 */
#define PERF_TIMED_SECTION_START(x) instrumentation_timedStart(x)
#define PERF_TIMED_SECTION_END(x) instrumentation_timedEnd(x)
#define PERF_MEASURE_PERIOD(x) instrumentation_trackPeriod(x)
#define PERF_TRACK_VALUE(x, y) instrumentation_updateCounter(x, y)
#define PERF_INCREMENT_VALUE(x) instrumentation_incrementCounter(x, 1)
#define PERF_DECREMENT_VALUE(x) instrumentation_incrementCounter(x, -1)

#else

#define PERF_DECLARE_COUNTER(x)
#define PERF_DEFINE_COUNTER(x)
#define PERF_USE_EXTERNAL_COUNTER(x)
#define PERF_SET_MODULE_ID(x)
#define PERF_INIT_COUNTER(x, id)
#define PERF_TIMED_SECTION_START(x)
#define PERF_TIMED_SECTION_END(x)
#define PERF_MEASURE_PERIOD(x)
#define PERF_TRACK_VALUE(x, y) (void)y
#define PERF_INCREMENT_VALUE(x)
#define PERF_DECREMENT_VALUE(x)
#endif /* INCLUDE_INSTRUMENTATION */
#endif /* instrumentation_HELPER_H */