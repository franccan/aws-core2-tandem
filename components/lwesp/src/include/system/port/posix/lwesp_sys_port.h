/**
 * \file            lwesp_sys_port.h
 * \brief           POSIX based system file implementation
 */

/*
 * Copyright (c) 2020 Francisco Gonzalez
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of LwESP - Lightweight ESP-AT parser library.
 *
 * Author:          Francisco Gonzalez <franccan@amazon.com>
 * Version:         $_version_$
 */
#ifndef LWESP_HDR_SYSTEM_PORT_H
#define LWESP_HDR_SYSTEM_PORT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Include any OS specific features */
#include <stdint.h>
#include <stdlib.h>
#include "lwesp/lwesp_opt.h"

#if LWESP_CFG_OS && !__DOXYGEN__


#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>

/**
 * \brief           System mutex type
 *
 * It is used by middleware as base type of mutex.
 */
typedef struct mutex_wrapper
{
    uint8_t is_valid;
    pthread_mutex_t mutex;
} mutex_wrapper_t;
typedef mutex_wrapper_t        lwesp_sys_mutex_t;

/**
 * \brief           System semaphore type
 *
 * It is used by middleware as base type of mutex.
 */
typedef struct sem_wrapper
{
    uint8_t is_valid;
    sem_t sem;
} sem_wrapper_t;
typedef sem_wrapper_t     lwesp_sys_sem_t;

/**
 * \brief           System message queue type
 *
 * It is used by middleware as base type of mutex.
 */
typedef struct mq_wrapper
{
	mqd_t mq;
	char name[30];
}mq_wrapper_t;
typedef mq_wrapper_t  lwesp_sys_mbox_t;

/**
 * \brief           System thread ID type
 */
typedef pthread_t        lwesp_sys_thread_t;

/**
 * \brief           System thread priority type
 *
 * It is used as priority type for system function,
 * to start new threads by middleware.
 */
typedef int          lwesp_sys_thread_prio_t;

/**
 * \brief           Mutex invalid value
 *
 * Value assigned to \ref gsm_sys_mutex_t type when it is not valid.
 */
#define LWESP_SYS_MUTEX_NULL          ((lwesp_sys_mutex_t)0)

/**
 * \brief           Semaphore invalid value
 *
 * Value assigned to \ref gsm_sys_sem_t type when it is not valid.
 */
#define LWESP_SYS_SEM_NULL            ((lwesp_sys_sem_t)0)

/**
 * \brief           Message box invalid value
 *
 * Value assigned to \ref gsm_sys_mbox_t type when it is not valid.
 */
#define LWESP_SYS_MBOX_NULL           ((lwesp_sys_mbox_t)-1)

/**
 * \brief           OS timeout value
 *
 * Value returned by operating system functions (mutex wait, sem wait, mbox wait)
 * when it returns timeout and does not give valid value to application
 */
#define LWESP_SYS_TIMEOUT             ((uint32_t)-1)

/**
 * \brief           Default thread priority value used by middleware to start built-in threads
 *
 * Threads can well operate with normal (default) priority and do not require
 * any special feature in terms of priority for prioer operation.
 */
#define LWESP_SYS_THREAD_PRIO         (0)

/**
 * \brief           Stack size in units of bytes for system threads
 *
 * It is used as default stack size for all built-in threads.
 */
#define LWESP_SYS_THREAD_SS           (4096)

#endif /* LWESP_CFG_OS && !__DOXYGEN__ */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LWESP_HDR_SYSTEM_PORT_H */
