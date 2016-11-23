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

#include "util/assert.h"

#include "infra/log.h"

#include "cfw/cfw_service.h"

#include "machine.h"
#include "storage.h"
#include "drivers/soc_flash.h"
#include "drivers/spi_flash.h"
#include "services/services_ids.h"
#include "util/cir_storage_flash_spi.h"
#include "circular_storage_service_private.h"
#include "services/circular_storage_service/circular_storage_service.h"

/****************************************************************************************
************************** SERVICE INITIALIZATION **************************************
****************************************************************************************/

static void handle_message(struct cfw_message *msg, void *param);
static void circular_storage_shutdown(service_t *svc, struct cfw_message *msg);

static service_t circular_storage_service = {
	.service_id = CIRCULAR_STORAGE_SERVICE_ID,
	.shutdown_request = circular_storage_shutdown,
};

#define MSG_ID_LL_CIRCULAR_STORAGE_SHUTDOWN_REQ 0xff00

static void circular_storage_shutdown(service_t *svc, struct cfw_message *msg)
{
	struct cfw_message *sm = (struct cfw_message *)message_alloc(
		sizeof(*sm), NULL);

	/* In order to ensure that any pending requests are processed prior to the
	 * service shutdown, send a message to self so it will be processed after
	 * all other pending requests.
	 */
	CFW_MESSAGE_ID(sm) = MSG_ID_LL_CIRCULAR_STORAGE_SHUTDOWN_REQ;
	CFW_MESSAGE_DST(sm) = circular_storage_service.port_id;
	CFW_MESSAGE_SRC(sm) = circular_storage_service.port_id;
	CFW_MESSAGE_PRIV(sm) = msg;
	cfw_send_message(sm);
}

/*
 * Init and Configure partitions seen by the Circular Storage Service.
 */
static void circular_storage_service_init(int id, void *queue)
{
	uint16_t flash_id = 0;
	int16_t partition_index = -1;
	flash_device_t flash;
	uint32_t i = 0;
	uint32_t j = 0;
	uint16_t partition_id;
	uint32_t first_block = 0;
	uint32_t block_count = 0;
	uint32_t elt_size = 0;

	/* for each storage configuration */
	for (i = 0; i < cir_storage_config.cir_storage_count; i++) {
		partition_id =
			cir_storage_config.cir_storage_list[i].partition_id;
		first_block =
			cir_storage_config.cir_storage_list[i].first_block;
		block_count =
			cir_storage_config.cir_storage_list[i].block_count;
		elt_size = cir_storage_config.cir_storage_list[i].element_size;
		/* Check storage configuration is valid */
		for (j = 0; j < cir_storage_config.no_part; j++) {
			if (cir_storage_config.partitions[j].partition_id ==
			    partition_id) {
				flash_id =
					cir_storage_config.partitions[j].
					flash_id;
				partition_index = j;
				break;
			}
		}
		assert(partition_index != -1);
		extern const flash_device_t flash_devices[];
		flash = flash_devices[flash_id];

		assert(flash.flash_location == SERIAL_FLASH);
		/* Initialise the configuration */
		cir_storage_config.cir_storage_list[i].storage =
			(void *)cir_storage_flash_spi_init(elt_size,
							   first_block,
							   block_count);
	}

	cfw_register_service(queue, &circular_storage_service, handle_message,
			     NULL);
}

CFW_DECLARE_SERVICE(circular_storage, CIRCULAR_STORAGE_SERVICE_ID,
		    circular_storage_service_init);


/*******************************************************************************
 *********************** SERVICE IMPLEMENTATION ********************************
 ******************************************************************************/
void handle_get(struct cfw_message *msg)
{
	circular_storage_get_req_msg_t *req =
		(circular_storage_get_req_msg_t *)msg;
	circular_storage_service_get_rsp_msg_t *resp =
		(circular_storage_service_get_rsp_msg_t *)cfw_alloc_rsp_msg(
			msg,
			MSG_ID_CIRCULAR_STORAGE_SERVICE_GET_RSP,
			sizeof(*resp));
	DRIVER_API_RC ret = DRV_RC_FAIL;
	uint32_t i = 0;

	for (i = 0; i < cir_storage_config.cir_storage_count; i++) {
		if (cir_storage_config.cir_storage_list[i].key == req->key) {
			if (cir_storage_config.cir_storage_list[i].storage !=
			    NULL) {
				ret = DRV_RC_OK;
				resp->storage =
					cir_storage_config.cir_storage_list[i].
					storage;
			} else {
				pr_debug(
					LOG_MODULE_MAIN,
					"Circular storage init: Flash SPI init failed");
				ret = DRV_RC_FAIL;
			}
		}
	}

	resp->status = ret;
	cfw_send_message(resp);
}

void handle_push(struct cfw_message *msg)
{
	circular_storage_push_req_msg_t *req =
		(circular_storage_push_req_msg_t *)msg;
	circular_storage_service_push_rsp_msg_t *resp =
		(circular_storage_service_push_rsp_msg_t *)cfw_alloc_rsp_msg(
			msg,
			MSG_ID_CIRCULAR_STORAGE_SERVICE_PUSH_RSP,
			sizeof(*resp));
	DRIVER_API_RC ret = DRV_RC_FAIL;

	if (cir_storage_push((cir_storage_t *)req->storage, req->buffer) ==
	    CBUFFER_STORAGE_SUCCESS) {
		ret = DRV_RC_OK;
	}

	resp->status = ret;
	cfw_send_message(resp);
}

void handle_pop(struct cfw_message *msg)
{
	circular_storage_pop_req_msg_t *req =
		(circular_storage_pop_req_msg_t *)msg;
	circular_storage_service_pop_rsp_msg_t *resp =
		(circular_storage_service_pop_rsp_msg_t *)cfw_alloc_rsp_msg(
			msg,
			MSG_ID_CIRCULAR_STORAGE_SERVICE_POP_RSP,
			sizeof(*resp));
	DRIVER_API_RC ret = DRV_RC_FAIL;

	resp->buffer = balloc(((cir_storage_t *)req->storage)->elt_size, NULL);
	if (cir_storage_pop((cir_storage_t *)req->storage, resp->buffer) ==
	    CBUFFER_STORAGE_SUCCESS) {
		ret = DRV_RC_OK;
	}

	resp->status = ret;
	cfw_send_message(resp);
}

void handle_peek(struct cfw_message *msg)
{
	circular_storage_peek_req_msg_t *req =
		(circular_storage_peek_req_msg_t *)msg;
	circular_storage_service_peek_rsp_msg_t *resp =
		(circular_storage_service_peek_rsp_msg_t *)cfw_alloc_rsp_msg(
			msg,
			MSG_ID_CIRCULAR_STORAGE_SERVICE_PEEK_RSP,
			sizeof(*resp));
	DRIVER_API_RC ret = DRV_RC_FAIL;
	int err;

	resp->buffer = balloc(((cir_storage_t *)req->storage)->elt_size, NULL);
	err = cir_storage_peek((cir_storage_t *)req->storage, resp->buffer);
	if (err == CBUFFER_STORAGE_SUCCESS) {
		ret = DRV_RC_OK;
	} else if (err == CBUFFER_STORAGE_EMPTY_ERROR) {
		ret = DRV_RC_OUT_OF_MEM;
	}

	resp->status = ret;
	cfw_send_message(resp);
}

void handle_clear(struct cfw_message *msg)
{
	circular_storage_clear_req_msg_t *req =
		(circular_storage_clear_req_msg_t *)msg;
	circular_storage_service_clear_rsp_msg_t *resp =
		(circular_storage_service_clear_rsp_msg_t *)cfw_alloc_rsp_msg(
			msg,
			MSG_ID_CIRCULAR_STORAGE_SERVICE_CLEAR_RSP,
			sizeof(*resp));

	if (cir_storage_clear((cir_storage_t *)req->storage,
			      req->elt_count) == CBUFFER_STORAGE_SUCCESS)
		resp->status = DRV_RC_OK;
	else
		resp->status = DRV_RC_FAIL;

	cfw_send_message(resp);
}

static void handle_message(struct cfw_message *msg, void *param)
{
	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_CIRCULAR_STORAGE_PUSH_REQ:
		handle_push(msg);
		break;
	case MSG_ID_CIRCULAR_STORAGE_POP_REQ:
		handle_pop(msg);
		break;
	case MSG_ID_CIRCULAR_STORAGE_PEEK_REQ:
		handle_peek(msg);
		break;
	case MSG_ID_CIRCULAR_STORAGE_CLEAR_REQ:
		handle_clear(msg);
		break;
	case MSG_ID_LL_CIRCULAR_STORAGE_SHUTDOWN_REQ:
		cfw_send_message(CFW_MESSAGE_PRIV(msg));
		break;
	case MSG_ID_CIRCULAR_STORAGE_GET_REQ:
		handle_get(msg);
		break;
	default:
		cfw_print_default_handle_error_msg(LOG_MODULE_MAIN,
						   CFW_MESSAGE_ID(
							   msg));
		break;
	}

	cfw_msg_free(msg);
}
