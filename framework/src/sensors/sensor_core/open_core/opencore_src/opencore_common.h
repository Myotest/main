/****************************************************************************************
 *
 * BSD LICENSE
 *
 * Copyright(c) 2015 Intel Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * * Neither the name of Intel Corporation nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************************/
#ifndef __OPENCORE_COMMON_H__
#define __OPENCORE_COMMON_H__

#include "infra/pm.h"
#include "infra/time.h"
#include "infra/log.h"
#include "infra/port.h"
#include "machine.h"
#include "ipc_comm.h"
#include "opencore_algo_common.h"
#include "sensors/phy_sensor_api/phy_sensor_api.h"

#ifdef SUSPEND_TEST
#include "drivers/bmi160_bus.h"
#endif

#define MATCH_BUFFER_LIMIT_SIZE     128
#define RAW_DATA_BUFFER_LIMIT_SIZE  1024
#define RAW_DATA_DUMP_BUFFER_SIZE   256
#define POLLING_TOLERANCE           10
#define ON                    (1 << 0)
#define IDLE                  (1 << 1)
#define SUBSCRIBED            (1 << 2)
#define WAKE_UP_CLEAR_FIFO    (1 << 3)
#define IGNORE                (1 << 1)

#define PHY_TYPE_KEY_LENGTH  3
#define PHY_ID_KEY_LENGTH    2

#define RAW_DATA_HEAD_CNT    2
#define FOREVER_VALUE ~((uint32_t)0)

#define DIRECT_RAW (1 << 1)
//#define SUPPORT_INTERRUPT_MODE

typedef struct {
	struct message m;
	void *priv;
}algo_engine_msg_t;

typedef enum {
	SYNC = 1,
	ASYNC,
}match_t;

/** algorithm request cmd for sensor_core*/
enum ipc_cmd_other2core_type {
	CMD_SUSPEND_SC = CMD_SVC2CORE_MAX,
	CMD_RESUME_SC,
	CMD_DOUBLE_TAP_SC,
	CMD_CALIBRATION_PROCESS_RESULT,
	CMD_RAWDATA_REG_INT_SC,
	CMD_RAWDATA_FIFO_INT_SC,
	CMD_REFRESH_SENSOR_CORE_SC,
	CMD_FIFO_READ_ONESHOT_OVER_SC,
	CMD_FIFO_CLEAR_CHECK_SC,
};

typedef enum {
	ALGO_PROCESS = 1,
	ALGO_GOTO_IDLE,
	ALGO_OUT_IDLE,
	READ_FIFO,
	CLEAR_FIFOS,
	CLOSE_PHY_SENSOR,
}algo_handle_t;

typedef struct {
	list_t raw_data_node;
	void *buffer;
	uint16_t raw_data_count;
}raw_data_node_t;

struct poll_links {
	list_t poll_link;
	list_t poll_active_link;
	list_t poll_active_array_link;
};

typedef struct {
	union {
		struct poll_links poll;
		list_t int_link;
	}links;
	list_t fifo_share_link;
	list_head_t raw_data_head[RAW_DATA_HEAD_CNT];

	sensor_t ptr;
	phy_sensor_type_t type;
	dev_id_t id;
	uint16_t stat_flag;

	int32_t range_max;
	int32_t range_min;
	uint32_t fifo_share_bitmap;
	uint16_t fifo_length;
	uint8_t sensor_data_frame_size;
	uint8_t sensor_data_raw_size;

	double npp;
	float pi;
	float pi_used_balloc;
	uint16_t freq;

	int16_t buffer_length;
	void *clb_data_buffer;
	void *feed_data_buffer;
	void *buffer;
	T_MUTEX mutex;

	uint8_t attri_mask;
	uint8_t idle_ref;
	uint8_t head_for_algo : 1;
	uint8_t dirty : 1;
	uint8_t fifo_use_flag : 1;
	uint8_t need_poll : 1;
	uint8_t fifo_share_done_flag : 1;
	uint8_t fifo_share_read_sync_done : 1;
}sensor_handle_t;

void *AllocFromDss(uint32_t size);
int FreeInDss(void *buf);

DEFINE_LOG_MODULE(LOG_MODULE_OPEN_CORE, "OCOR")

#endif
