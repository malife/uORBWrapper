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
 * @file publisher.c
 *
 * This file is part of the implementation of the example presented in the
 *  interprocess communication described in 
 *  http://pixhawk.org/dev/shared_object_communication
 */

// #include "topic.h"

#include "uORB.h"

#include "random_integer.h" 

/* create topic metadata */
ORB_DEFINE(random_integer,struct _random_integer); //dash in struct name is to comply with LCM

#include <unistd.h>
 
/* file handle that will be used for publishing */
static int topic_handle;
 
int
init()
{
    /* generate the initial data for first publication */
    struct _random_integer rd = { .r = random(), };
 
    /* advertise the topic and make the initial publication */
    topic_handle = orb_advertise(ORB_ID(random_integer), &rd);
    return 1;
}
 
int
update_topic()
{
    /* generate a new random number for publication */
    struct _random_integer rd = { .r = random(), };
 
    /* publish the new data structure */
    orb_publish(ORB_ID(random_integer), topic_handle, &rd);
    printf("value = %d\n", rd.r);

    return 0;
}

int main(int argc, char const *argv[])
{
    /* code */
    init();
    uint32_t i;

    for(i=0; i<5000; i++) {
        update_topic();

        sleep(1);
    }

    return 0;
}