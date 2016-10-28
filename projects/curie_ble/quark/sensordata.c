/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
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

#include <string.h>

#include "util/misc.h"
#include "os/os.h"
#include "infra/log.h"

#include "cfw/cfw.h"

/* Main sensors API */
#include "services/sensor_service/sensor_service.h"

#include "drivers/data_type.h"
#include "sensordata.h"
#include "infra/time.h"
#include "lib/ble/rscs/ble_rscs.h"

#define STEP_LENGTH          50 /* Arbitrary step length in cms */

/* Sensors client */
static cfw_service_conn_t *sensor_service_conn = NULL;
/* Sensor handles */
static sensor_service_t accel_handle = NULL;

/* handle for sensors data */
static void handle_sensor_subscribe_data(struct cfw_message *msg)
{
	sensor_service_subscribe_data_event_t *p_evt =
		(sensor_service_subscribe_data_event_t *)msg;
	sensor_service_sensor_data_header_t *p_data_header =
		&p_evt->sensor_data_header;
	uint8_t sensor_type = GET_SENSOR_TYPE(p_evt->handle);

	switch (sensor_type) {
	case SENSOR_ABS_CADENCE:;
		struct cadence_result *p =
			(struct cadence_result *)p_data_header->data;
		/* New value of step cadence sensor */
		/* BLE RSC profile reports stride cadence whereas the sensor reports
		 * step cadence => need to convert the sensor value before BLE notification:
		 * 1 stride = 2 steps */
		pr_info(LOG_MODULE_MAIN, "stride cadence=%d", p->cadence / 2);
		ble_rscs_update(0, p->cadence / 2);
		break;
	}
}

static void handle_start_scanning_evt(struct cfw_message *msg)
{
	sensor_service_scan_event_t *p_evt = (sensor_service_scan_event_t *)msg;
	sensor_service_on_board_scan_data_t on_board_data =
		p_evt->on_board_data;
	uint8_t sensor_type = p_evt->sensor_type;

	/* Sensor exists; save its handle for further subscription */
	pr_info(LOG_MODULE_MAIN, "Detected cadence sensor");
	accel_handle = GET_SENSOR_HANDLE(sensor_type, on_board_data.ch_id);
}

static void sensor_handle_msg(struct cfw_message *msg, void *data)
{
	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_SENSOR_SERVICE_START_SCANNING_EVT:
		handle_start_scanning_evt(msg);
		break;
	case MSG_ID_SENSOR_SERVICE_SUBSCRIBE_DATA_EVT:
		handle_sensor_subscribe_data(msg);
		break;
	default: break;
	}
	cfw_msg_free(msg);
}

int on_ble_rscs_get_stride_lenth()
{
	int stride_length;

	stride_length = (STEP_LENGTH * 2);   /* 1 stride = 2 steps, 1 step = STEP_LENGTH */
	return stride_length;
}

void on_ble_rscs_enabled(void)
{
	uint8_t data_type = ACCEL_DATA;

	if (accel_handle) {
		pr_info(LOG_MODULE_MAIN, "Subscribe to cadence sensor");
		sensor_service_subscribe_data(sensor_service_conn, NULL,
					      accel_handle,
					      &data_type, 1, 1,
					      5000);
	}
}

void on_ble_rscs_disabled(void)
{
	uint8_t data_type = ACCEL_DATA;

	if (accel_handle) {
		pr_info(LOG_MODULE_MAIN, "Unsubscribe from cadence sensor");
		sensor_service_unsubscribe_data(sensor_service_conn, NULL,
						accel_handle, &data_type, 1);
	}
}

static void service_connection_cb(cfw_service_conn_t *conn, void *param)
{
	sensor_service_conn = conn;
	/* Scan for sensor presence */
	sensor_service_start_scanning(conn, NULL, CADENCE_TYPE_MASK);
}

void sensordata_init(T_QUEUE queue)
{
	/* Initialize client message handler */
	cfw_client_t *sensor_client = cfw_client_init(queue, sensor_handle_msg,
						      NULL);

	/* Open the sensor service */
	cfw_open_service_helper(sensor_client, ARC_SC_SVC_ID,
				service_connection_cb, NULL);
}
