#include "fcat_log.h"

#include <stdio.h>

static char fcat_log_name_[FCAT_LOG_NAME_MAX] = "fcat";

// Thread-local, so only the process thread ever sees a non-zero value and there
// is no cross-thread race. __thread, not _Thread_local, as this is built as gnu99.
static __thread int fcat_log_is_rt_thread_ = 0;

void fcat_log_set_name(const char* name)
{
  if (name == NULL || name[0] == '\0') {
    return;
  }
  // snprintf, not strncpy: always NUL-terminates and avoids
  // -Wstringop-truncation, which is fatal under jsd-lib's -Werror.
  snprintf(fcat_log_name_, FCAT_LOG_NAME_MAX, "%s", name);
}

const char* fcat_log_name(void) { return fcat_log_name_; }

void fcat_log_mark_rt_thread(void) { fcat_log_is_rt_thread_ = 1; }
int  fcat_log_throttle_enabled(void) { return fcat_log_is_rt_thread_; }
