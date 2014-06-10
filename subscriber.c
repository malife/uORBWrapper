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

/**
 * @file subscriber.c
 *
 * This file is part of the implementation of the example presented in the
 *  interprocess communication described in 
 *  http://pixhawk.org/dev/shared_object_communication
 */
#include "uORB.h"

#include "random_integer.h" 
ORB_DEFINE(random_integer, struct _random_integer);

// This code needs to be done automatically
// ===============================================================
struct _random_integer_handler_data_t {
    lcm_recv_buf_t rbuf;
    char* channel; 
    random_integer msg;
} random_integer_handler_data;


static void random_integer_handler (const lcm_recv_buf_t* rbuf, const char* channel, 
        const random_integer* msg, void* userdata) {
    memcpy(&(random_integer_handler_data.rbuf), rbuf, sizeof(lcm_recv_buf_t));
    
    random_integer_handler_data.channel = (char *) malloc(sizeof(channel));
    memcpy(&(random_integer_handler_data.channel), channel, sizeof(channel));

    memcpy(&(random_integer_handler_data.msg), msg, sizeof(random_integer));
}

// ===============================================================
 
/* file handle that will be used for subscribing */
static int topic_handle;
 
int
init()
{
    /* subscribe to the topic */
    topic_handle = orb_subscribe(ORB_ID(random_integer));
}
 
void
check_topic()
{
    bool updated;
    struct _random_integer rd;
 
    /* check to see whether the topic has updated since the last time we read it */
    orb_check(topic_handle, &updated);
 
    if (updated) {
        /* make a local copy of the updated data structure */
        orb_copy(ORB_ID(random_integer), topic_handle, &rd);
        printf("Random integer is now %d\n", rd.r);
    }
}


int main(int argc, char const *argv[])
{
    init();
    uint64_t i = 0;

    for (i=0; i <50000; i++){
        check_topic();
        usleep(250);
    }


    return 0;
}

