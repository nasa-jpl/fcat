#ifndef FCAT__JSD_PRINT_H_
#define FCAT__JSD_PRINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fcat_log.h"

#include <jsd/jsd_time.h>
#include <stdio.h>

#ifndef FCAT_MAX_LOG_LEVEL
#define FCAT_MAX_LOG_LEVEL FCAT_LOG_LEVEL_INFO
#endif

#if FCAT_MAX_LOG_LEVEL >= FCAT_LOG_LEVEL_ERROR
#define ERROR(M, ...)                                                          \
  do {                                                                         \
    if (fcat_log_get_level() >= FCAT_LOG_LEVEL_ERROR)                          \
      fprintf(stderr, "\033[1;31m[ ERROR ] [%lf] (%s:%d) " M "\033[0m\n",     \
              jsd_time_get_time_sec(), __FILE__, __LINE__, ##__VA_ARGS__);      \
  } while (0)
#else
#define ERROR(M, ...) do {} while (0)
#endif

#if FCAT_MAX_LOG_LEVEL >= FCAT_LOG_LEVEL_WARN
#define WARNING(M, ...)                                                        \
  do {                                                                         \
    if (fcat_log_get_level() >= FCAT_LOG_LEVEL_WARN)                           \
      fprintf(stderr, "\033[1;33m[ WARN  ] [%lf] (%s:%d) " M "\033[0m\n",     \
              jsd_time_get_time_sec(), __FILE__, __LINE__, ##__VA_ARGS__);      \
  } while (0)
#else
#define WARNING(M, ...) do {} while (0)
#endif

#if FCAT_MAX_LOG_LEVEL >= FCAT_LOG_LEVEL_INFO
#define MSG(M, ...)                                                            \
  do {                                                                         \
    if (fcat_log_get_level() >= FCAT_LOG_LEVEL_INFO)                           \
      fprintf(stderr, "[ INFO  ] [%lf] (%s:%d) " M "\n",                      \
              jsd_time_get_time_sec(), __FILE__, __LINE__, ##__VA_ARGS__);      \
  } while (0)
#define SUCCESS(M, ...)                                                        \
  do {                                                                         \
    if (fcat_log_get_level() >= FCAT_LOG_LEVEL_INFO)                           \
      fprintf(stderr, "\033[1;32m[SUCCESS] [%lf] (%s:%d) " M "\033[0m\n",     \
              jsd_time_get_time_sec(), __FILE__, __LINE__, ##__VA_ARGS__);      \
  } while (0)
#else
#define MSG(M, ...) do {} while (0)
#define SUCCESS(M, ...) do {} while (0)
#endif

#if FCAT_MAX_LOG_LEVEL >= FCAT_LOG_LEVEL_DEBUG
#define MSG_DEBUG(M, ...)                                                      \
  do {                                                                         \
    if (fcat_log_get_level() >= FCAT_LOG_LEVEL_DEBUG)                          \
      fprintf(stderr, "[ DEBUG ] [%lf] (%s:%d) " M "\n",                      \
              jsd_time_get_time_sec(), __FILE__, __LINE__, ##__VA_ARGS__);      \
  } while (0)
#else
#define MSG_DEBUG(M, ...) do {} while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif
