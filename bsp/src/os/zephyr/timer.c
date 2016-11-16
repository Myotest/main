/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* ZEPHYR OS abstraction / timer services */

#include <limits.h>

#include <zephyr.h>
#include "os/os.h"
#include "infra/panic.h"
#include "common.h"
#include "infra/time.h"

/*type for timer status */
typedef enum {
	E_TIMER_READY = 0,
	E_TIMER_RUNNING
} T_TIMER_STATUS;

/** Timer descriptor */
typedef struct {
	T_ENTRY_POINT callback; /* function to call on timer expiration -- a NULL value means the timer is not running */
	void *data;           /* data to provide to the callback */
	uint32_t expiration;    /* tick when timer is due to expire */
	uint32_t delay;         /* timer "timeout" in us -- used for repeating timers */
	uint8_t repeat;      /* specifies if timer shall be automatically restarted upon expiration */
	uint8_t status;      /* describe the timer state */
} T_TIMER_DESC;

/** Chained list of timers */
typedef struct _timer_list {
	T_TIMER_DESC desc;
	struct _timer_list *prev;
	struct _timer_list *next;
}T_TIMER_LIST_ELT;



/**
 * Return the current tick, converted to microseconds.
 *   Tick is read on HW timer.
 *
 * MicroK:
 *     According to tick.h (line 69), ISR_HighTimerRead is task_node_cycle_get_32.
 *     Hence, this function always calls task_node_cycle_get_32, whatever the execution
 *     context.
 *     ticktime is set by nanotime.c
 *
 * NanoK:
 *     Neither the programmer's guide nor the source code state limitations
 *     related to the execution context concerning nano_node_cycle_get_32.
 *     ticktime and tickunit are set by NODE1.c
 *
 */


/**********************************************************
************** Private variables  ************************
**********************************************************/
#define TIMER_CBK_TASK_STACK_SIZE   1024   /* Size in bytes of the stack for the Timer callback task -- NOT USED BY ZEPHYR MICROKERNEL */
#define TIMER_CBK_TASK_PRIORITY        1   /* Priority of the Timer callback task  -- NOT USED BY ZEPHYR MICROKERNEL */
#define TIMER_CBK_TASK_OPTIONS         0   /* Fiber options for the Timer callback task - 0 -> floating point is not supported  -- NOT USED BY ZEPHYR MICROKERNEL */

#ifdef CONFIG_NANOKERNEL
/** Stack for Timer fiber */
static uint32_t g_TimerFiberStack [TIMER_CBK_TASK_STACK_SIZE /
				   sizeof(uint32_t)];
/** Single nanokernel semaphore used to implement all provided timers */
static struct nano_sem g_TimerSem;

#else /* CONFIG_MICROKERNEL */
/** Semaphore used for the timer callbacks */
static ksem_t g_TimerSem;

/** Definition of the private task "OS_TASK_TIMER" */
#include "microkernel/task.h"
DEFINE_TASK(OS_TASK_TIMER, 5, timer_task_port, TIMER_CBK_TASK_STACK_SIZE, 0);

/**
 * This function is there for compatibility issue between the original
 * "timer_task" definition and the requested prototype by the macro DEFINE_TASK
 * from the file "microkernel/task.h"
 */
void timer_task_port(void)
{
	extern void timer_task(int dummy1, int dummy2);
	timer_task(0, 0);
}
#endif

/** Pool of timers */
DECLARE_BLK_ALLOC(g_TimerPool, T_TIMER_LIST_ELT, TIMER_POOL_SIZE) /* see common.h */

/** Head of the chained list that sorts active timers according to their expiration date */
T_TIMER_LIST_ELT * g_CurrentTimerHead;

/**********************************************************
************** Forward declarations **********************
**********************************************************/
static void signal_timer_task(void);
static bool is_after_expiration(uint32_t tick, T_TIMER_DESC *tmrDesc);
static void add_timer(T_TIMER_LIST_ELT *newTimer);
static void remove_timer(T_TIMER_LIST_ELT *timerToRemove);
static void execute_callback(T_TIMER_LIST_ELT *expiredTimer);

void timer_task(int dummy1, int dummy2);

/**********************************************************
************** Private functions  ************************
**********************************************************/
/**
 * Signal g_TimerSem to wake-up timer_task.
 *
 */
static void signal_timer_task(void)
{
#ifdef   CONFIG_NANOKERNEL
	nano_sem_give(&g_TimerSem);
#else
	T_EXEC_LEVEL execLvl;

	execLvl = _getExecLevel();
	/* call the nanoK service that corresponds to the current execution level */
	switch (execLvl) {
	case E_EXEC_LVL_ISR:
		isr_sem_give(g_TimerSem);
		break;

	case E_EXEC_LVL_FIBER:
		fiber_sem_give(g_TimerSem);
		break;

	case E_EXEC_LVL_TASK:
		task_sem_give(g_TimerSem);
		break;
	default: /* will not do that from unknown context */
		panic(E_OS_ERR_NOT_SUPPORTED);
		break;
	}
#endif
}


#ifdef __DEBUG_OS_ABSTRACTION_TIMER
/**
 *  Print the list of active timers, from tail (first to expire) to head (last to expire).
 */
static void display_list(void)
{
	T_TIMER_LIST_ELT *tbv;

	tbv = g_CurrentTimerHead;
	if (NULL == tbv) {
		_log(" list is empty");
	} else {
		_log(" Head = 0x%x", (uint32_t)tbv);
		while (NULL != tbv) {
			_log(" -> 0x%x", (uint32_t)tbv->next);
			tbv = tbv->next;
		}
	}
}
#endif

/**
 * Returns whether the provided tick
 *     is before or after the expiration
 *     of a timer.
 *
 * @param tick date to compare
 * @param tmrDesc timer descriptor
 *
 * @return true if tick parameter represents
 *     a date after the expiration of the
 *     timer
 *     or false if the timer descriptor is null
 *     or if tick represents a date before the
 *     expiration of the timer
 *
 */
static bool is_after_expiration(uint32_t tick, T_TIMER_DESC *tmrDesc)
{
	if (NULL != tmrDesc) {
		/* timer will expire before the tick rolls over */
		return tick >= tmrDesc->expiration;
	}
	return false;
}


/**
 * Insert a timer in the list of active timer,
 *    according to its expiration date.
 *
 * @param newTimer pointer on the timer to insert
 *
 * WARNING: newTimer MUST NOT be null (rem: static function )
 *
 */
static void add_timer(T_TIMER_LIST_ELT *newTimer)
{
	T_TIMER_LIST_ELT *insertionPoint;
	T_TIMER_LIST_ELT *listTail;
	bool found;

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
	_log(
		"\nINFO : add_timer - start: adding 0x%x to expire at %d (now = %d - delay = %d - ticktime = %d)",
		(uint32_t)newTimer, newTimer->desc.expiration,
		get_uptime_ms(), newTimer->desc.delay, sys_clock_us_per_tick);
	display_list();
#endif


	if (NULL == g_CurrentTimerHead) {
		/* there is no active timer, make newTimer the head timer */
		g_CurrentTimerHead = newTimer;
		newTimer->prev = NULL;
		newTimer->next = NULL;
	} else {
		/* find the next timer to expire after newTimer->desc.expiration */
		insertionPoint = g_CurrentTimerHead;
		listTail = g_CurrentTimerHead;
		found = false;
		while ((NULL != insertionPoint) && (false == found)) {
			if (!is_after_expiration(newTimer->desc.expiration,
						 &(insertionPoint->desc))) { /* newTimer is due to expire before insertionPoint */
				if (g_CurrentTimerHead == insertionPoint) { /*  insert newTimer before the head timer  */
					g_CurrentTimerHead->prev = newTimer;
					newTimer->prev = NULL;
					newTimer->next = g_CurrentTimerHead;
					g_CurrentTimerHead = newTimer;
				} else { /* insert newTimer between insertionPoint and insertionPoint->prev */
					newTimer->prev = insertionPoint->prev;
					newTimer->next = insertionPoint;
					insertionPoint->prev = newTimer;
					newTimer->prev->next = newTimer;
				}
				/* done */
				found = true;
			} else { /* newTimer is due to expire after insertionPoint, continue searching */
				listTail = insertionPoint; /* update list head marker = last non null insertionPoint */
				/* move downward */
				insertionPoint = insertionPoint->next;
			}
		}

		if ((false == found) && (NULL != listTail)) {
			/* newTimer is due to expire after all others have expired */
			listTail->next = newTimer;
			newTimer->prev = listTail;
		}
	}

	newTimer->desc.status = E_TIMER_RUNNING;

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
	_log("\nINFO : add_timer - end "); display_list();
	display_list();
#endif
}

/**
 * Remove a timer from the list of active timers.
 *
 * @param timerToRemove pointer on the timer to remove
 *
 * WARNING: timerToRemove MUST NOT be null (rem: static function )
 *
 */
static void remove_timer(T_TIMER_LIST_ELT *timerToRemove)
{
	T_TIMER_LIST_ELT *removePoint;

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
	_log(
		"\nINFO : remove_timer - start: removing 0x%x to expire at %d (now = %d)",
		(uint32_t)timerToRemove, timerToRemove->desc.expiration,
		get_uptime_ms());
	display_list();
#endif

	if (NULL != g_CurrentTimerHead) {
		removePoint = g_CurrentTimerHead;
		while (NULL != removePoint) {
			if (timerToRemove == removePoint) {
				if (NULL != timerToRemove->next) {
					timerToRemove->next->prev =
						timerToRemove->prev;
				}
				if (NULL != timerToRemove->prev) {
					timerToRemove->prev->next =
						timerToRemove->next;
				}
			}
			removePoint = removePoint->next;
		}

		if (timerToRemove == g_CurrentTimerHead) {
			g_CurrentTimerHead = g_CurrentTimerHead->next;
			if (NULL != g_CurrentTimerHead) {
				g_CurrentTimerHead->prev = NULL;
			}
		}
	} else {
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
		_log("\nERROR : remove_timer : list of active timer is empty ");
#endif
		panic(E_OS_ERR);
	}

	/* clean-up links */
	timerToRemove->prev = NULL;
	timerToRemove->next = NULL;
	timerToRemove->desc.status = E_TIMER_READY;


#ifdef __DEBUG_OS_ABSTRACTION_TIMER
	_log("\nINFO : remove_timer - end ");
	display_list();
#endif
}


/**
 * Execute the callback of a timer.
 *
 * @param expiredTimer pointer on the timer
 *
 * WARNING: expiredTimer MUST NOT be null (rem: static function )
 */
static void execute_callback(T_TIMER_LIST_ELT *expiredTimer)
{
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
	_log(
		"\nINFO : execute_callback : executing callback of timer 0x%x  (now = %u - expiration = %u)",
		(uint32_t)expiredTimer,
		get_uptime_ms(), expiredTimer->desc.expiration);
#endif
	int flags = irq_lock();

	/* if the timer was not stopped by its own callback */
	if (E_TIMER_RUNNING == expiredTimer->desc.status) {
		remove_timer(expiredTimer);
		/* add it again if repeat flag was on */
		if (expiredTimer->desc.repeat) {
			expiredTimer->desc.expiration = get_uptime_ms() +
							expiredTimer->desc.
							delay;
			add_timer(expiredTimer);
		}
	}
	irq_unlock(flags);

	/* call callback back */
	if (NULL != expiredTimer->desc.callback) {
		expiredTimer->desc.callback(expiredTimer->desc.data);
	} else {
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
		_log("\nERROR : execute_callback : timer callback is null ");
#endif
		panic(E_OS_ERR);
	}
}

/**********************************************************
************** Exported functions ************************
**********************************************************/

/*----- Initialization  */

/**
 * Initialize the resources used by the framework's timer services
 *
 * IMPORTANT : this function must be called during the initialization
 *             of the OS abstraction layer.
 *             this function shall only be called once after reset, otherwise
 *             it may cause the take/lock and give/unlock services to fail
 */
void os_init_timer(void)
{
	uint8_t idx;

#ifdef CONFIG_NANOKERNEL
	nano_sem_init(&g_TimerSem);
#else
	g_TimerSem = OS_TIMER_SEM;
#endif

	/* start with empty list of active timers: */
	g_CurrentTimerHead = NULL;

	/* memset ( g_TimerPool_elements, 0 ):  */
	for (idx = 0; idx < TIMER_POOL_SIZE; idx++) {
		g_TimerPool_elements[idx].desc.callback = NULL;
		g_TimerPool_elements[idx].desc.data = NULL;
		g_TimerPool_elements[idx].desc.delay = 0;
		g_TimerPool_elements[idx].desc.expiration = 0;
		g_TimerPool_elements[idx].desc.repeat = false;
		g_TimerPool_elements[idx].prev = NULL;
		g_TimerPool_elements[idx].next = NULL;
		/* hopefully, the init function is performed before
		 *  timer_create and timer_stop can be called,
		 *  hence there is no need for a critical section
		 *  here */
	}

	/* start "timer_task" in a new fiber for NanoK, or a new task for microK */
#ifdef CONFIG_NANOKERNEL
	fiber_fiber_start((char *)g_TimerFiberStack, TIMER_CBK_TASK_STACK_SIZE,
			  timer_task, 0, 0,
			  TIMER_CBK_TASK_PRIORITY, TIMER_CBK_TASK_OPTIONS);
#else
	task_start(OS_TASK_TIMER);
#endif
}

/**
 * Return the current tick converted in milliseconds.
 *
 * Authorized execution levels:  task, fiber
 *
 * @warning: the return value is false in interrupt context.
 *
 * @return current tick converted in milliseconds
 */
uint32_t get_time_ms(void)
{
	return (uint32_t)CONVERT_TICKS_TO_MS(sys_tick_get_32());
}

/**
 * Return the current tick converted in microseconds.
 *
 * Authorized execution levels:  task, fiber
 *
 * ZEPHYR does not provide an API to read a high-precision timer, that
 * returns a 64-bits value.
 * The tick is read as a 32-bits value, then casted to 64-bits.
 *
 * @warning: the return value is false in interrupt context.
 *
 * @return current tick converted in microseconds
 */
uint64_t get_time_us(void)
{
	return (uint64_t)CONVERT_TICKS_TO_US(sys_tick_get_32());
}

/**
 * Create  a timer object.
 *     This service may panic if err parameter is null and:
 *         callback parameter is null, or
 *         no timer is available.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * @param callback: pointer to the function to be executed.
 * @param privData: pointer to data that shall be passed to the callback
 * @param delay: number of milliseconds between function executions
 * @param repeat: specifies if the timer shall be re-started after each execution of the callback
 * @param startup : specifies if the timer shall be start immediately
 * @param err (out): execution status:
 *         E_OS_OK : callback is programmed
 *         E_OS_ERR: no timer is available, or callback parameter is null
 *
 * @return Handler on the timer, NULL if the service fails (e.g. no available
 *         timer or callback is a null pointer).
 */
T_TIMER timer_create(T_ENTRY_POINT callback, void *privData, uint32_t delay,
		     bool repeat, bool startup,
		     OS_ERR_TYPE *err)
{
	T_TIMER_LIST_ELT *timer = NULL;

	/* check input parameters */
	if ((NULL != callback) && (OS_WAIT_FOREVER != delay)) {
		/* delay should be set to 0 if startup flag is false
		 * otherwise delay should be a positive value if startup flag is true */
		if (((0 < delay) &&
		     (true == startup)) ||
		    ((0 <= delay) && (false == startup))) {
			/* find and reserve a timer resource from g_TimerPool_elements */
			/* rem: timer points to an element from the global g_TimerPool_elements */
			timer = g_TimerPool_alloc();
			if (timer != NULL) {
				/* initialize timer descriptor */
				timer->desc.callback = callback;
				timer->desc.data = privData;
				timer->desc.delay = delay;
				timer->desc.repeat = repeat;
				timer->desc.status = E_TIMER_READY;

				/* insert timer in the list of active timers */
				if (startup) {
					int flags;
					timer->desc.expiration =
						get_uptime_ms() +
						timer->desc.delay;
					flags = irq_lock();
					add_timer(timer);
					irq_unlock(flags);
					if (g_CurrentTimerHead == timer) {
						/* new timer is the next to expire, unblock timer_task to assess the change */
						signal_timer_task();
					}
				}

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
				_log(
					"\nINFO : timer_create : new timer will expire at %u (now = %u ) - addr = 0x%x",
					timer->desc.expiration,
					get_uptime_ms(), (uint32)timer);
#endif
				error_management(err, E_OS_OK);
			} else {
				/* all timers from the pool are already being used */
				error_management(err, E_OS_ERR_NO_MEMORY);
			}
		} else { /* delay and startup parameter are inconsistent */
			error_management(err, E_OS_ERR);
		}
	} else { /* callback == NULL or delay == 0 : at least one parameter is invalid */
		error_management(err, E_OS_ERR);
	}
	return (T_TIMER)timer;
}

/**
 * start the timer.
 *     This service may panic if err parameter is null and:
 *         no timer is available or timer is running .
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * @param tmr : handler on the timer (value returned by timer_create ).
 * @param err (out): execution status:
 */
void timer_start(T_TIMER tmr, uint32_t delay, OS_ERR_TYPE *err)
{
	T_TIMER_LIST_ELT *timer = (T_TIMER_LIST_ELT *)tmr;
	OS_ERR_TYPE localErr = E_OS_OK;

	/* if timer is created */
	if (NULL != timer) {
		int flags = irq_lock();
		if (timer->desc.status == E_TIMER_READY) {
			/* if timer parameter are valid */
			if ((NULL != timer->desc.callback) && (0 < delay)) {
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
				_log("\nINFO : timer_start : starting  timer ");
#endif
				/* Update expiration time */
				timer->desc.delay = delay;
				timer->desc.expiration = get_uptime_ms() +
							 timer->desc.delay;
				/* add the timer */
				add_timer(timer);

				irq_unlock(flags);
				/* new timer is the next to expire, unblock timer_task to assess the change */
				if (g_CurrentTimerHead == timer) {
					signal_timer_task();
				}
			} else {
				/* timer is not valid */
				localErr = E_OS_ERR;
			}
		} else if (timer->desc.status == E_TIMER_RUNNING) {
			irq_unlock(flags);
			localErr = E_OS_ERR_BUSY;
			if (err != NULL)
				*err = localErr;
			return;
		}
	} else { /* tmr is not a timer from g_TimerPool_elements */
		localErr = E_OS_ERR;
	}

	error_management(err, localErr);
}

/**
 *  Remove the timer in Chained list of timers.
 *     This service may panic if:
 *         tmr parameter is is null, invalid, or timer is not running.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * @param tmr : handler on the timer (value returned by timer_create ).
 *
 */
void timer_stop(T_TIMER tmr)
{
	T_TIMER_LIST_ELT *timer = (T_TIMER_LIST_ELT *)tmr;
	bool doSignal = false;

	if (NULL != timer) {
		int flags = irq_lock();
		/* if timer is active */
		if (timer->desc.status == E_TIMER_RUNNING) {
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
			_log(
				"\nINFO : timer_stop : stopping timer at addr = 0x%x",
				(uint32_t)timer);
#endif
			/* remove the timer */

			if (g_CurrentTimerHead == timer) {
				doSignal = true;
			}

			remove_timer(timer);

			irq_unlock(flags);

			if (doSignal) {
				/* the next timer to expire was removed, unblock timer_task to assess the change */
				signal_timer_task();
			}
		} else { /* tmr is not running */
			irq_unlock(flags);
		}
	} else { /* tmr is not a timer from g_TimerPool_elements */
		panic(E_OS_ERR);
	}
}

/**
 * Delete the timer object.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * @param tmr : handler on the timer (value returned by timer_create ).
 *
 */
void timer_delete(T_TIMER tmr)
{
	T_TIMER_LIST_ELT *timer = (T_TIMER_LIST_ELT *)tmr;

	if (NULL != timer) {
		/* check if timer is running and stop it */
		int flags = irq_lock();
		if (timer->desc.status == E_TIMER_RUNNING) {
			timer_stop(timer);
		}
		irq_unlock(flags);

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
		_log("\nINFO : timer_delete : deleting  timer at addr = 0x%x",
		     (uint32)timer);
#endif

		/*  free the timer   */
		g_TimerPool_free(timer);
	} else { /* tmr is not a timer from g_TimerPool_elements */
		panic(E_OS_ERR);
	}
}

/**
 * This function is called when the CPU knows the next occurence when it will
 * need to be awoken after a deepsleep.
 * It can be used by slaves CPUs when they need to tell a master CPU when it
 * can switch the SOC to deepsleep, like in the Quark SE SOC */
void __attribute__ ((weak)) publish_cpu_timeout(uint32_t timeout_ms)
{
}

/**
 * Main function of the timer task. This function is in charge of
 *    calling the callbacks associated with the timers.
 *
 *    This function is run in a high priority task on microkernel builds.
 *    This function is run in a high priority fiber on nanokernel builds.
 *    It implements an infinite loop that waits for any of the semaphores from
 *    g_TimerSem.
 *    When a semaphore is signaled, this function fetches the pointer to the
 *    associated callback in g_TimerDesc, then calls it.
 *
 * @param dummy1 not used (required by ZEPHYR API)
 * @param dummy2 not used (required by ZEPHYR API) *
 *
 */
void timer_task(int dummy1, int dummy2)
{
	int32_t timeout = UINT32_MAX;
	uint32_t now;

	UNUSED(dummy1);
	UNUSED(dummy2);

	while (1) { /* the Timer task shall never stop */
		/* Before waiting for next timeout, publish it to warn QUARK cpu */
		publish_cpu_timeout(timeout);
		/* block until g_TimerSem is signaled or until the next timeout expires */
#ifdef CONFIG_MICROKERNEL
		(void)task_sem_take(g_TimerSem, CONVERT_MS_TO_TICKS(timeout));
#else
		nano_sem_take(&g_TimerSem, CONVERT_MS_TO_TICKS(timeout));
#endif
		now = get_uptime_ms();
		/* task is unblocked: check for expired timers */
		while (g_CurrentTimerHead &&
		       is_after_expiration(now, &(g_CurrentTimerHead->desc))) {
			execute_callback(g_CurrentTimerHead);
			now = get_uptime_ms();
		}
		/* Compute timeout until the expiration of the next timer */
		if (g_CurrentTimerHead != NULL) {
			/* In micro kernel context, timeout = 0 or timeout < 0 works.
			 * In nano kernel context timeout must be a positive value.
			 */
			timeout = g_CurrentTimerHead->desc.expiration - now;
			if (timeout < 0) panic(E_OS_ERR_OVERFLOW);
		} else {
			timeout = UINT32_MAX;
		}

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
		if (NULL != g_CurrentTimerHead)
			_log(
				"\nINFO : timer_task : now = %u, next timer expires at %u, timeout = %u",
				get_uptime_ms(),
				g_CurrentTimerHead->desc.expiration,
				timeout);
		else
			_log(
				"\nINFO : timer_task : now = %u, no next timer, timeout = OS_WAIT_FOREVER",
				get_uptime_ms());
#endif
	} /* end while(1) */
}
