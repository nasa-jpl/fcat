#include "fcat_log.h"

static int fcat_log_level_ = FCAT_LOG_LEVEL_WARN;

void fcat_log_set_level(int level) { fcat_log_level_ = level; }
int  fcat_log_get_level(void)      { return fcat_log_level_; }
