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

#include <zephyr.h>

#include "util/assert.h"

#include "os/os.h"
#include "infra/xloop.h"
#include "infra/system_events.h"
#include "cfw/cfw.h"
#include "services/services_ids.h"

/* Definition of the private task "TASK_STORAGE" */
DEFINE_TASK(TASK_STORAGE, 7, storage_task, 2048, 0);

static T_QUEUE storage_queue;
xloop_t loop;

/* Storage initialisation */
void init_storage(void)
{
	storage_queue = queue_create(10);
	assert(storage_queue);

#ifdef CONFIG_SERVICES_QUARK_SE_LL_STORAGE_IMPL
	cfw_set_queue_for_service(LL_STOR_SERVICE_ID, storage_queue);
#endif
#ifdef CONFIG_SERVICES_QUARK_SE_CIRCULAR_STORAGE_IMPL
	cfw_set_queue_for_service(CIRCULAR_STORAGE_SERVICE_ID, storage_queue);
#endif
#ifdef CONFIG_SERVICES_QUARK_SE_PROPERTIES_IMPL
	cfw_set_queue_for_service(PROPERTIES_SERVICE_ID, storage_queue);
#endif

	/* The task storage will init low level storage service,
	 * property service and circular storage service */
	task_start(TASK_STORAGE);
}

void storage_task(void)
{
	cfw_loop(storage_queue);
#ifdef CONFIG_SYSTEM_EVENTS
	system_event_set_xloop(&loop);
#endif
	xloop_run(&loop);
}
