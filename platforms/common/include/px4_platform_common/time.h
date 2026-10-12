#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <time.h>
#include <pthread.h>

#if defined(__PX4_POSIX)
__BEGIN_DECLS
__EXPORT int px4_clock_gettime(clockid_t clk_id, struct timespec *tp);
__END_DECLS
#else
#define px4_clock_gettime system_clock_gettime
#endif

#if defined(ENABLE_LOCKSTEP_SCHEDULER) || defined(__PX4_QURT)

__BEGIN_DECLS
__EXPORT int px4_clock_settime(clockid_t clk_id, const struct timespec *tp);
__END_DECLS

#else

#define px4_clock_settime system_clock_settime

#endif

__BEGIN_DECLS
/**
 * Sleep for usec microseconds of wall-clock time, outside of the lockstep scheduler.
 *
 * Any duration works, whereas usleep() only has to accept less than a second
 * (and POSIX.1-2008 removed it). As with usleep(), a signal ends the sleep early.
 *
 * @return 0, or -1 with errno set (EINTR if a signal interrupted the sleep)
 */
__EXPORT int system_usleep(useconds_t usec);
__END_DECLS

#if defined(ENABLE_LOCKSTEP_SCHEDULER)

__BEGIN_DECLS
__EXPORT int px4_usleep(useconds_t usec);
__EXPORT unsigned int px4_sleep(unsigned int seconds);
__EXPORT int px4_pthread_cond_timedwait(pthread_cond_t *cond,
					pthread_mutex_t *mutex,
					const struct timespec *abstime);
__END_DECLS

#else

#define px4_usleep system_usleep
#define px4_sleep system_sleep
#define px4_pthread_cond_timedwait system_pthread_cond_timedwait

#endif
