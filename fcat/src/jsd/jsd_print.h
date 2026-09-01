#ifndef FCAT__JSD_PRINT_H_
#define FCAT__JSD_PRINT_H_

// Shadow of jsd's jsd_print.h, selected over it by the include order set in
// CMakeLists.txt. Upstream these macros fprintf(stderr), which reaches the
// terminal but never the rcl log file or /rosout; routing them through rcutils
// puts them in every sink the node's own RCLCPP_* calls reach.

// Kept for jsd/fastcat sources that get jsd_time and snprintf transitively from
// this header (e.g. jsd_egd.c).
#include <jsd/jsd_time.h>
#include <stdio.h>

#include "fcat_log.h"
#include "rcutils/logging_macros.h"
#include "rcutils/time.h"

// The call site is embedded in the message text because the default output
// format renders no file or line. __FILE_NAME__ (gcc >= 12) is the basename;
// __FILE__ is an absolute build path.
#ifdef __FILE_NAME__
#define FCAT_LOG_FILE __FILE_NAME__
#else
#define FCAT_LOG_FILE __FILE__
#endif

// Compile-time ceiling; anything above it is removed from the binary. Runtime
// filtering is left to rcutils, which RCUTILS_LOG_*_NAMED already consults.
#ifndef FCAT_MAX_LOG_LEVEL
#define FCAT_MAX_LOG_LEVEL FCAT_LOG_LEVEL_INFO
#endif

#ifndef FCAT_LOG_THROTTLE_MS
#define FCAT_LOG_THROTTLE_MS 1000
#endif

// Throttles only on the realtime process thread, the one place a fault can
// reassert every cycle. MSG/SUCCESS/MSG_DEBUG are never throttled: fastcat logs
// device lists from inside loops, so throttling them would emit the first entry
// and drop the rest.
#if FCAT_LOG_THROTTLE_MS > 0
#define FCAT_LOG_MAYBE_THROTTLED(SEVERITY, M, ...)                             \
  do {                                                                         \
    if (fcat_log_throttle_enabled()) {                                         \
      RCUTILS_LOG_##SEVERITY##_THROTTLE_NAMED(                                 \
          RCUTILS_STEADY_TIME, FCAT_LOG_THROTTLE_MS, fcat_log_name(),           \
          "(%s:%d) " M, FCAT_LOG_FILE, __LINE__, ##__VA_ARGS__);                \
    } else {                                                                   \
      RCUTILS_LOG_##SEVERITY##_NAMED(fcat_log_name(), "(%s:%d) " M,             \
                                     FCAT_LOG_FILE, __LINE__, ##__VA_ARGS__);   \
    }                                                                          \
  } while (0)
#else
#define FCAT_LOG_MAYBE_THROTTLED(SEVERITY, M, ...)                             \
  RCUTILS_LOG_##SEVERITY##_NAMED(fcat_log_name(), "(%s:%d) " M, FCAT_LOG_FILE,  \
                                 __LINE__, ##__VA_ARGS__)
#endif

#if FCAT_MAX_LOG_LEVEL >= FCAT_LOG_LEVEL_ERROR
#define ERROR(M, ...) FCAT_LOG_MAYBE_THROTTLED(ERROR, M, ##__VA_ARGS__)
#else
#define ERROR(M, ...) \
  do {                \
  } while (0)
#endif

#if FCAT_MAX_LOG_LEVEL >= FCAT_LOG_LEVEL_WARN
#define WARNING(M, ...) FCAT_LOG_MAYBE_THROTTLED(WARN, M, ##__VA_ARGS__)
#else
#define WARNING(M, ...) \
  do {                  \
  } while (0)
#endif

#if FCAT_MAX_LOG_LEVEL >= FCAT_LOG_LEVEL_INFO
#define MSG(M, ...)                                                           \
  RCUTILS_LOG_INFO_NAMED(fcat_log_name(), "(%s:%d) " M, FCAT_LOG_FILE,        \
                         __LINE__, ##__VA_ARGS__)
// rcutils has no SUCCESS severity, so keep the distinction in the text.
#define SUCCESS(M, ...)                                                       \
  RCUTILS_LOG_INFO_NAMED(fcat_log_name(), "(%s:%d) SUCCESS: " M,              \
                         FCAT_LOG_FILE, __LINE__, ##__VA_ARGS__)
#else
#define MSG(M, ...) \
  do {              \
  } while (0)
#define SUCCESS(M, ...) \
  do {                  \
  } while (0)
#endif

#if FCAT_MAX_LOG_LEVEL >= FCAT_LOG_LEVEL_DEBUG
#define MSG_DEBUG(M, ...)                                                     \
  RCUTILS_LOG_DEBUG_NAMED(fcat_log_name(), "(%s:%d) " M, FCAT_LOG_FILE,       \
                          __LINE__, ##__VA_ARGS__)
#else
#define MSG_DEBUG(M, ...) \
  do {                    \
  } while (0)
#endif

#endif
