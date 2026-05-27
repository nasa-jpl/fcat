#ifndef FCAT_LOG_H_
#define FCAT_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define FCAT_LOG_LEVEL_ERROR 1
#define FCAT_LOG_LEVEL_WARN  2
#define FCAT_LOG_LEVEL_INFO  3
#define FCAT_LOG_LEVEL_DEBUG 4

void fcat_log_set_level(int level);
int  fcat_log_get_level(void);

#ifdef __cplusplus
}
#endif

#endif
