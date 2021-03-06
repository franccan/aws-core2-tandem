
#include "system/lwesp_sys.h"
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

static lwesp_sys_mutex_t sys_mutex;


#define NSEC_PER_SEC 1000000000

/**
 * \brief           Init system dependant parameters
 *
 * After this function is called,
 * all other system functions must be fully ready.
 *
 * \return          `1` on success, `0` otherwise
 */
uint8_t

lwesp_sys_init(void) {
    return lwesp_sys_mutex_create(&sys_mutex);
}

/**
 * \brief           Convert milliseconds to timespec
 * \param[in]       msec: milliseconds value
 * \param[out]      ts: out timesepec value
 */
void
msec_to_timespec(unsigned long msec, struct timespec *ts)
{
    if (msec < 1000){
        ts->tv_sec = 0;
        ts->tv_nsec = msec * 1000000;
    }
    else {
        ts->tv_sec = msec / 1000;
        ts->tv_nsec = (msec - ts->tv_sec * 1000) * 1000000;
    }
}

/**
 * \brief           Convert timespec to ms
 * \param[in]       msec: timesepec value
 * \return          milliseconds value
 */
uint32_t
timespec_to_msec(struct timespec *ts)
{
    long ms;    //milliseconds
    time_t s;   //seconds
    s = ts->tv_sec;
    ms = round(ts->tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
    if (ms > 999) {
        s++;
        ms = 0;
    }
    return ((s * 1000) + ms);
}

/* Add a nanosecond value to a timespec
 *
 * \param r[out] result: a + b
 * \param a[in] base operand as timespec
 * \param b[in] operand in nanoseconds
 */
void
timespec_add_nsec(struct timespec *r, const struct timespec *a, int64_t b)
{
    r->tv_sec = a->tv_sec + (b / NSEC_PER_SEC);
    r->tv_nsec = a->tv_nsec + (b % NSEC_PER_SEC);

    if (r->tv_nsec >= NSEC_PER_SEC) {
        r->tv_sec++;
        r->tv_nsec -= NSEC_PER_SEC;
    } else if (r->tv_nsec < 0) {
        r->tv_sec--;
        r->tv_nsec += NSEC_PER_SEC;
    }
}

/* Add a millisecond value to a timespec
 *
 * \param r[out] result: a + b
 * \param a[in] base operand as timespec
 * \param ms[in] operand in milliseconds
 */
void
timespec_add_msec(struct timespec *r, const struct timespec *a, int64_t ms)
{
    timespec_add_nsec(r, a, ms * 1000000);
}

/* Subtract timespecs
 *
 * \param r[out] result: a - b
 * \param a[in] operand
 * \param b[in] operand
 */
void
timespec_sub(struct timespec *r,
         const struct timespec *a, const struct timespec *b)
{
    r->tv_sec = a->tv_sec - b->tv_sec;
    r->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (r->tv_nsec < 0) {
        r->tv_sec--;
        r->tv_nsec += NSEC_PER_SEC;
    }
}

/**
 * \brief           Get current time in units of milliseconds
 * \return          Current time in units of milliseconds
 */
uint32_t
lwesp_sys_now(void) {
    struct timespec spec;
    clock_gettime(CLOCK_MONOTONIC, &spec);
    return timespec_to_msec(&spec);
}

/**
 * \brief           Protect middleware core
 *
 * Stack protection must support recursive mode.
 * This function may be called multiple times,
 * even if access has been granted before.
 *
 * \note            Most operating systems support recursive mutexes.
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_protect(void) {
    return lwesp_sys_mutex_unlock(&sys_mutex);
}

/**
 * \brief           Unprotect middleware core
 *
 * This function must follow number of calls of \ref lwesp_sys_protect
 * and unlock access only when counter reached back zero.
 *
 * \note            Most operating systems support recursive mutexes.
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_unprotect(void) {
    return lwesp_sys_mutex_unlock(&sys_mutex);
}



/**
 * \brief           Create new recursive mutex
 * \note            Recursive mutex has to be created as it may be locked multiple times before unlocked
 * \param[out]      p: Pointer to mutex structure to allocate
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mutex_create(lwesp_sys_mutex_t* p) {
    pthread_mutexattr_t attr;
    int ret;

    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    ret = pthread_mutex_init(&p->mutex, &attr);
    p->is_valid = (ret == 0);
    return p->is_valid;
}



/**
 * \brief           Delete recursive mutex from system
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mutex_delete(lwesp_sys_mutex_t* p) {
    int ret;
    if(!p->is_valid)
        return 0;

    ret = pthread_mutex_destroy(&p->mutex);
    p->is_valid = 0;
    return ret == 0;
}

/**
 * \brief           Lock recursive mutex, wait forever to lock
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mutex_lock(lwesp_sys_mutex_t* p) {
    if(!p->is_valid)
        return 0;

    return pthread_mutex_lock(&p->mutex) == 0;
}

/**
 * \brief           Unlock recursive mutex
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mutex_unlock(lwesp_sys_mutex_t* p) {
    if(! p->is_valid)
        return 0;

    return pthread_mutex_unlock(&p->mutex) == 0;

}

/**
 * \brief           Check if mutex structure is valid system
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mutex_isvalid(lwesp_sys_mutex_t* p) {
    return p != NULL && p->is_valid;
}

/**
 * \brief           Set recursive mutex structure as invalid
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mutex_invalid(lwesp_sys_mutex_t* p) {
    p->is_valid = 0;
    return 1;
}

/**
 * \brief           Create a new binary semaphore and set initial state
 * \note            Semaphore may only have `1` token available
 * \param[out]      p: Pointer to semaphore structure to fill with result
 * \param[in]       cnt: Count indicating default semaphore state:
 *                     `0`: Take semaphore token immediately
 *                     `1`: Keep token available
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_sem_create(lwesp_sys_sem_t* p, uint8_t cnt) {
    int shared_between_processes = 0;
    int result = sem_init(&p->sem, shared_between_processes, cnt);
    printf("cnt: %i\r\n",cnt);
    p->is_valid = result == 0;
    return p->is_valid;
}

/**
 * \brief           Delete binary semaphore
 * \param[in]       p: Pointer to semaphore structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_sem_delete(lwesp_sys_sem_t* p) {
    if(!p->is_valid)
        return 0;
    int result = sem_destroy(&p->sem);
    p->is_valid = 0;
    return result == 0;
}

/**
 * \brief           Wait for semaphore to be available
 * \param[in]       p: Pointer to semaphore structure
 * \param[in]       timeout: Timeout to wait in milliseconds. When `0` is applied, wait forever
 * \return          Number of milliseconds waited for semaphore to become available or
 *                      \ref LWESP_SYS_TIMEOUT if not available within given time
 */
uint32_t
lwesp_sys_sem_wait(lwesp_sys_sem_t* p, uint32_t timeout) {
    printf("lwesp_sys_sem_wait...0\r\n");
    if(timeout == 0) {  //if no timeout
      printf("lwesp_sys_sem_wait...000000\r\n");
        return sem_wait(&p->sem);
    }
    printf("lwesp_sys_sem_wait...1\r\n");

    //we have a timeout
    struct timespec before_call, deadline, after_call, elapsed;
    printf("lwesp_sys_sem_wait...2\r\n");
    clock_gettime(CLOCK_MONOTONIC, &before_call);   //get time before call
    printf("lwesp_sys_sem_wait...3\r\n");
    timespec_add_msec(&deadline, &before_call, timeout);    //compute deadline
    printf("lwesp_sys_sem_wait...4\r\n");
    if( sem_timedwait(&p->sem, &deadline) != 0) {   //wait on semaphore
        return LWESP_SYS_TIMEOUT; //if timeout
    }
    printf("lwesp_sys_sem_wait...5\r\n");
    clock_gettime(CLOCK_MONOTONIC, &after_call);    //get time after call
    printf("lwesp_sys_sem_wait...6\r\n");
    timespec_sub(&elapsed, &after_call, &before_call);
    printf("lwesp_sys_sem_wait...7\r\n");
    return timespec_to_msec(&elapsed);
}

/**
 * \brief           Release semaphore
 * \param[in]       p: Pointer to semaphore structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_sem_release(lwesp_sys_sem_t* p) {

    int ret = sem_post(&p->sem);
    ret = sem_close(&p->sem);
    
    int value;
    sem_getvalue(&p->sem, &value);
    printf("&lwesp_sys_sem_release: %d\r\n", value );

    p->is_valid = 0;
    return ret == 0;
}

/**
 * \brief           Check if semaphore is valid
 * \param[in]       p: Pointer to semaphore structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_sem_isvalid(lwesp_sys_sem_t* p) {
    return p != NULL && p->is_valid;
}

/**
 * \brief           Invalid semaphore
 * \param[in]       p: Pointer to semaphore structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_sem_invalid(lwesp_sys_sem_t* p) {
    p->is_valid = 0;
    return 1;
}

/**
 * \brief           Create a new message queue with entry type of `void *`
 * \param[out]      b: Pointer to message queue structure
 * \param[in]       size: Number of entries for message queue to hold
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mbox_create(lwesp_sys_mbox_t* b, size_t size) {
    static uint32_t suffix = 0;
    pid_t pid;

    //build message queue name
    pid = getpid();
    snprintf(b->name, sizeof(b->name), "/%ld_%u", (long int)pid, suffix );
    ++suffix;   //can loop

    //create it
    struct mq_attr mqattr;
    mqattr.mq_maxmsg = size;
    mqattr.mq_msgsize = sizeof(void*);
    b->mq = mq_open(b->name, O_RDWR | O_CREAT | O_EXCL, S_IRWXU, &mqattr);
    return b->mq != -1;
}

/**
 * \brief           Delete message queue
 * \param[in]       b: Pointer to message queue structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mbox_delete(lwesp_sys_mbox_t* b) {
    if(mq_close(b->mq) == -1)
        return 0;
    if(mq_unlink(b->name) == -1)
        return 0;
    return 1;
}

/**
 * \brief           Put a new entry to message queue and wait until memory available
 * \param[in]       b: Pointer to message queue structure
 * \param[in]       m: Pointer to entry to insert to message queue
 * \return          Time in units of milliseconds needed to put a message to queue
 */
uint32_t
lwesp_sys_mbox_put(lwesp_sys_mbox_t* b, void* m) {
    struct timespec before_call, after_call, elapsed;
    clock_gettime(CLOCK_MONOTONIC, &before_call);   //get time before call

    if( mq_send(b->mq, m, sizeof(m), 0) == -1)
        return LWESP_SYS_TIMEOUT;

    clock_gettime(CLOCK_MONOTONIC, &after_call);    //get time after call
    timespec_sub(&elapsed, &after_call, &before_call);
    return timespec_to_msec(&elapsed);
}

/**
 * \brief           Get a new entry from message queue with timeout
 * \param[in]       b: Pointer to message queue structure
 * \param[in]       m: Pointer to pointer to result to save value from message queue to
 * \param[in]       timeout: Maximal timeout to wait for new message. When `0` is applied, wait for unlimited time
 * \return          Time in units of milliseconds needed to put a message to queue
 *                      or \ref LWESP_SYS_TIMEOUT if it was not successful
 */
uint32_t
lwesp_sys_mbox_get(lwesp_sys_mbox_t* b, void** m, uint32_t timeout) {
    struct timespec before_call, deadline, after_call, elapsed;
    ssize_t received_size;
    unsigned int received_prio;
    clock_gettime(CLOCK_MONOTONIC, &before_call);   //get time before call

    if(timeout == 0) {  //if no timeout
        received_size = mq_receive(b->mq, *m, sizeof(void*), &received_prio);
    }
    else {
        timespec_add_msec(&deadline, &before_call, timeout);    //compute deadline
        received_size = mq_timedreceive(b->mq, *m, sizeof(void*), &received_prio, &deadline);
    }
    clock_gettime(CLOCK_MONOTONIC, &after_call);    //get time after call

    if(received_size == -1) {
        return LWESP_SYS_TIMEOUT;
    }
    timespec_sub(&elapsed, &after_call, &before_call);
    return timespec_to_msec(&elapsed);
}

/**
 * \brief           Put a new entry to message queue without timeout (now or fail)
 * \param[in]       b: Pointer to message queue structure
 * \param[in]       m: Pointer to message to save to queue
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mbox_putnow(lwesp_sys_mbox_t* b, void* m) {
    struct timespec fake_deadline;
    memset(&fake_deadline, 0, sizeof(fake_deadline));
    return mq_timedsend(b->mq, m, sizeof(void*), 0, &fake_deadline) == 0 ? 1 : 0;
}

/**
 * \brief           Get an entry from message queue immediately
 * \param[in]       b: Pointer to message queue structure
 * \param[in]       m: Pointer to pointer to result to save value from message queue to
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mbox_getnow(lwesp_sys_mbox_t* b, void** m) {
    struct timespec fake_deadline;
    unsigned int prio;
    memset(&fake_deadline, 0, sizeof(fake_deadline));
    return mq_timedreceive(b->mq, *m, sizeof(void*), &prio, &fake_deadline) == -1 ? 0 : 1;
}

/**
 * \brief           Check if message queue is valid
 * \param[in]       b: Pointer to message queue structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mbox_isvalid(lwesp_sys_mbox_t* b) {
    return b != NULL && b->mq != -1;
}

/**
 * \brief           Invalid message queue
 * \param[in]       b: Pointer to message queue structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_mbox_invalid(lwesp_sys_mbox_t* b) {
    b->mq = -1;
    return 1;
}


typedef struct thread_wrapper
{
    lwesp_sys_thread_fn func;
    void* args;
    lwesp_sys_sem_t start_flag;   //the semaphore is used to indicate that the new thread has retrieve all usefull informations from its arg so the structure can be destroyed
} thread_wrapper_t;

/**
 * \brief           Function wrapper to create posix threads
 * \param[in]       args = pointer to thread_wrapper_t
 * \return          NULL
 * \note            posix thread function must have the following signature : void*(void*)
 * \note            whereas lwesp_sys_thread_fn has the following signture : void(void*)
 */
static void*
thread_func_wrapper(void* args)
{
    thread_wrapper_t* th_wrapper = (thread_wrapper_t*)args; //cast
    lwesp_sys_thread_fn thread_func = th_wrapper->func;   //copy useful informations
    void* thread_args = th_wrapper->args;    //copy useful informations

    lwesp_sys_sem_release(&th_wrapper->start_flag);  //notify that we did the copy
    printf("thread_func_wrapper!!! 2\r\n");
    thread_func(thread_args);
    return NULL;
}



/**
 * \brief           Create a new thread
 * \param[out]      t: Pointer to thread identifier if create was successful.
 *                     It may be set to `NULL`
 * \param[in]       name: Name of a new thread
 * \param[in]       thread_func: Thread function to use as thread body
 * \param[in]       arg: Thread function argument
 * \param[in]       stack_size: Size of thread stack in uints of bytes. If set to 0, reserve default stack size
 * \param[in]       prio: Thread priority
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_thread_create(lwesp_sys_thread_t* t, const char* name, lwesp_sys_thread_fn thread_func,
                      void* const arg, size_t stack_size, lwesp_sys_thread_prio_t prio) {
    pthread_attr_t attr;
    thread_wrapper_t wrapper;
    printf("lwesp_sys_thread_create 1\r\n");
    if(pthread_attr_init(&attr) != 0) {
        return 0;
    }
    printf("lwesp_sys_thread_create 2\r\n");
    if(stack_size != 0 && stack_size > PTHREAD_STACK_MIN && pthread_attr_setstacksize(&attr, stack_size) != 0) {
        pthread_attr_destroy(&attr);
        return 0;
    }
    printf("lwesp_sys_thread_create 3\r\n");
    struct sched_param priority;
    priority.sched_priority = prio;
    if(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0 || pthread_attr_setschedparam(&attr, &priority) != 0 ) {
        pthread_attr_destroy(&attr);
        return 0;
    }
    printf("lwesp_sys_thread_create 4\r\n");
    //Create the wrapper for posix thread
    wrapper.func = thread_func;
    wrapper.args = arg;
    if(lwesp_sys_sem_create(&wrapper.start_flag, 0) != 1){
        pthread_attr_destroy(&attr);
        return 0;
    }

    int value;
    sem_getvalue(&wrapper.start_flag.sem, &value);
    printf("&wrapper.start_flag: %d\r\n", value );

    if(pthread_create(t, &attr, thread_func_wrapper, &wrapper) != 0) {
        pthread_attr_destroy(&attr);
        return 0;
    }
    //sleep(5);
    //Now we wait that the new thread has correctly started and recovered informations from args, so we can destroy args
    lwesp_sys_sem_wait(&wrapper.start_flag, 0);
    lwesp_sys_sem_release(&wrapper.start_flag);
    
    pthread_attr_destroy(&attr);
    //pthread_setname_np(*t, name);
    return 1;
}

/**
 * \brief           Terminate thread (shut it down and remove)
 * \param[in]       t: Pointer to thread handle to terminate.
 *                      If set to `NULL`, terminate current thread (thread from where function is called)
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_thread_terminate(lwesp_sys_thread_t* t) {
    if(t == NULL)
        return 0;

    if(pthread_join(*t, NULL) != 0)
        return 0;

    return 1;
}

/**
 * \brief           Yield current thread
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwesp_sys_thread_yield(void) {
    return sched_yield() == 0;
}