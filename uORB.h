/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef _UORB_UORB_H
#define _UORB_UORB_H

/**
 * @file uORB.h
 * API for the uORB lightweight object broker.
 */

#include <sys/types.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Hack until everything is using this header
// TODO: See if in PC wrapper this include must be correctly directed
#include "visibility.h"

/**
 * Object metadata.
 */
struct orb_metadata {
	const char *o_name;		/**< unique object name */
	const size_t o_size;		/**< object size */
    int (*publish)(lcm_t*, const char*, const void*); /** %$% Added in Wrapper*/
    void* (*subscribe)(lcm_t*, const char*, void (*handler)(), const void*); /** %$% Added in Wrapper*/
    void* (*handler)(const lcm_recv_buf_t* , const char*, const void*, void *); /** %$% Added in Wrapper*/
};

typedef const struct orb_metadata *orb_id_t;

/**
 * Generates a pointer to the uORB metadata structure for
 * a given topic.
 *
 * The topic must have been declared previously in scope
 * with ORB_DECLARE().
 *
 * @param _name		The name of the topic.
 */
#define ORB_ID(_name)		&__orb_##_name

/**
 * Declare (prototype) the uORB metadata for a topic.
 *
 * Note that optional topics are declared weak; this allows a potential
 * subscriber to attempt to subscribe to a topic that is not known to the
 * system at runtime.  The ORB_ID() macro will return NULL/nullptr for
 * such a topic, and attempts to advertise or subscribe to it will
 * return -1/ENOENT (see below).
 *
 * @param _name		The name of the topic.
 */
#if defined(__cplusplus)
# define ORB_DECLARE(_name)		extern "C" const struct orb_metadata __orb_##_name __EXPORT
// # define ORB_DECLARE_OPTIONAL(_name)    extern "C" const struct orb_metadata __orb_##_name __EXPORT __attribute__((weak))
# define ORB_DECLARE_OPTIONAL(_name)	extern "C" const struct orb_metadata __orb_##_name __EXPORT __WEAK
#else
# define ORB_DECLARE(_name)     extern const struct orb_metadata __orb_##_name __EXPORT
// # define ORB_DECLARE(_name)		extern const struct orb_metadata __orb_##_name __EXPORT
# define ORB_DECLARE_OPTIONAL(_name)	extern const struct orb_metadata __orb_##_name __EXPORT __WEAK
#endif

/**
 * Define (instantiate) the uORB metadata for a topic.
 *
 * The uORB metadata is used to help ensure that updates and
 * copies are accessing the right data.
 *
 * In the case of the uORBWrapper it also implements the 
 * handling structure and the actual function handler to receive data
 *
 * Note that there must be no more than one instance of this macro
 * for each topic.
 *
 * @param _name		The name of the topic.
 * @param _struct	The structure the topic provides.
 */
#define ORB_DEFINE(_name, _struct)			\
\
struct _##_name##_handler_data_t { \
    lcm_recv_buf_t rbuf; \
    char channel[40];  \
    _name msg; \
} _name##_handler_data; \
\
static void _name##_handler (const lcm_recv_buf_t* rbuf, const char* channel, \
        const _name* msg, void* userdata) { \
    memcpy(&(_name##_handler_data.rbuf), rbuf, sizeof(lcm_recv_buf_t)); \
    memcpy(&(_name##_handler_data.channel), channel, 39); \
    _name##_handler_data.channel[39] = '\0'; \
    memcpy(&(_name##_handler_data.msg), msg, sizeof(_name));\
}\
\
	const struct orb_metadata __orb_##_name = {	\
		#_name,					\
		sizeof(_struct),				\
        &_name##_publish,         \
        &_name##_subscribe,      \
        &_name##_handler \
	}; struct hack

__BEGIN_DECLS

/**
 * ORB topic advertiser handle.
 *
 * Advertiser handles are global; once obtained they can be shared freely
 * and do not need to be closed or released.
 *
 * This permits publication from interrupt context and other contexts where
 * a file-descriptor-based handle would not otherwise be in scope for the
 * publisher.
 */
typedef intptr_t	orb_advert_t;

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node in /obj if required and publishes the initial data.
 *
 * Any number of advertisers may publish to a topic; publications are atomic
 * but co-ordination between publishers is not provided by the ORB. 
 *
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.
 * @param data		A pointer to the initial data to be published.
 *			For topics updated by interrupt handlers, the advertisement
 *			must be performed from non-interrupt context.
 * @return		ERROR on error, otherwise returns a handle
 *			that can be used to publish to the topic.
 *			If the topic in question is not known (due to an
 *			ORB_DEFINE with no corresponding ORB_DECLARE)
 *			this function will return -1 and set errno to ENOENT.
 */
extern orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data) __EXPORT;

/**
 * Publish new data to a topic.
 *
 * The data is atomically published to the topic and any waiting subscribers
 * will be notified.  Subscribers that are not waiting can check the topic
 * for updates using orb_check and/or orb_stat.
 *
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.
 * @handle		The handle returned from orb_advertise.
 * @param data		A pointer to the data to be published.
 * @return		OK on success, ERROR otherwise with errno set accordingly.
 */
extern int	orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data) __EXPORT;

/**
 * Subscribe to a topic.
 *
 * The returned value is a file descriptor that can be passed to poll()
 * in order to wait for updates to a topic, as well as topic_read,
 * orb_check and orb_stat.
 *
 * Subscription will succeed even if the topic has not been advertised;
 * in this case the topic will have a timestamp of zero, it will never
 * signal a poll() event, checking will always return false and it cannot
 * be copied. When the topic is subsequently advertised, poll, check,
 * stat and copy calls will react to the initial publication that is
 * performed as part of the advertisement.
 *
 * Subscription will fail if the topic is not known to the system, i.e.
 * there is nothing in the system that has declared the topic and thus it
 * can never be published.
 *
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.
 * @return		ERROR on error, otherwise returns a handle
 *			that can be used to read and update the topic.
 *			If the topic in question is not known (due to an
 *			ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
 *			this function will return -1 and set errno to ENOENT.
 */
extern int	orb_subscribe(const struct orb_metadata *meta) __EXPORT;

/**
 * Unsubscribe from a topic.
 *
 * @param handle	A handle returned from orb_subscribe.
 * @return		OK on success, ERROR otherwise with errno set accordingly.
 */
extern int	orb_unsubscribe(int handle) __EXPORT;

/**
 * Fetch data from a topic.
 *
 * This is the only operation that will reset the internal marker that
 * indicates that a topic has been updated for a subscriber. Once poll
 * or check return indicating that an updaet is available, this call
 * must be used to update the subscription.
 *
 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
 *			for the topic.
 * @param handle	A handle returned from orb_subscribe.
 * @param buffer	Pointer to the buffer receiving the data, or NULL
 *			if the caller wants to clear the updated flag without
 *			using the data.
 * @return		OK on success, ERROR otherwise with errno set accordingly.
 */
extern int	orb_copy(const struct orb_metadata *meta, int handle, void *buffer) __EXPORT;

/**
 * Check whether a topic has been published to since the last orb_copy.
 *
 * This check can be used to determine whether to copy the topic when
 * not using poll(), or to avoid the overhead of calling poll() when the
 * topic is likely to have updated.
 *
 * Updates are tracked on a per-handle basis; this call will continue to
 * return true until orb_copy is called using the same handle. This interface
 * should be preferred over calling orb_stat due to the race window between
 * stat and copy that can lead to missed updates.
 *
 * @param handle	A handle returned from orb_subscribe.
 * @param updated	Set to true if the topic has been updated since the
 *			last time it was copied using this handle.
 * @return		OK if the check was successful, ERROR otherwise with
 *			errno set accordingly.
 */
extern int	orb_check(int handle, bool *updated) __EXPORT;

/**
 * Return the last time that the topic was updated.
 *
 * @param handle	A handle returned from orb_subscribe.
 * @param time		Returns the absolute time that the topic was updated, or zero if it has
 *			never been updated. Time is measured in microseconds.
 * @return		OK on success, ERROR otherwise with errno set accordingly.
 */
extern int	orb_stat(int handle, uint64_t *time) __EXPORT;

/**
 * Set the minimum interval between which updates are seen for a subscription.
 *
 * If this interval is set, the subscriber will not see more than one update
 * within the period.
 *
 * Specifically, the first time an update is reported to the subscriber a timer
 * is started. The update will continue to be reported via poll and orb_check, but
 * once fetched via orb_copy another update will not be reported until the timer
 * expires.
 *
 * This feature can be used to pace a subscriber that is watching a topic that
 * would otherwise update too quickly.
 *
 * @param handle	A handle returned from orb_subscribe.
 * @param interval	An interval period in milliseconds.
 * @return		OK on success, ERROR otherwise with ERRNO set accordingly.
 */
extern int	orb_set_interval(int handle, unsigned interval) __EXPORT;

__END_DECLS

#endif /* _UORB_UORB_H */
