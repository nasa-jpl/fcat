#ifndef FCAT_LOG_H_
#define FCAT_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define FCAT_LOG_LEVEL_ERROR 1
#define FCAT_LOG_LEVEL_WARN  2
#define FCAT_LOG_LEVEL_INFO  3
#define FCAT_LOG_LEVEL_DEBUG 4

#define FCAT_LOG_NAME_MAX 128

// Logger name for jsd/fastcat records. Pass this->get_logger().get_name() so
// they share the node's logger and follow its level at run time. Call once from
// the node constructor, before any other thread exists; NULL/empty is a no-op.
void fcat_log_set_name(const char* name);

// Never NULL. Read-only after the set above, so safe from the process thread.
const char* fcat_log_name(void);

// Mark the calling thread as the realtime process loop; call at the top of the
// process callback. Only ERROR/WARNING from a marked thread are throttled, so
// warnings fanned out over devices from callback threads stay intact.
void fcat_log_mark_rt_thread(void);
int  fcat_log_throttle_enabled(void);

#ifdef __cplusplus
}
#endif

#endif
