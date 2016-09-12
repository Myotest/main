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

#ifndef __OPENCORE_ALGO_COMMON_H__
#define __OPENCORE_ALGO_COMMON_H__
/**
 * @defgroup open_sensor_core Open Sensor Core API
 * Open core exposed api/structure
 * @ingroup sensor_core
 * @{
 */
#include "stddef.h"
#include "string.h"
#include "stdint.h"
#include "services/sensor_service/sensor_data_format.h"
#include "util/list.h"
#include "util/compiler.h"

#define DEFAULT_ID       (uint8_t)(~0)
#define DEFAULT_VALUE    (int16_t)(~0)

/**
 * this enum is used to feed type define
 **/
typedef enum {
	BASIC_ALGO_GESTURE = 1,
	BASIC_ALGO_STEPCOUNTER,
	BASIC_ALGO_TAPPING,
	BASIC_ALGO_SIMPLEGES,
	BASIC_ALGO_RAWDATA,
	BASIC_ALGO_OHRM,
	BASIC_ALGO_ALTITUDE,
	BASIC_ALGO_KB,
	BASIC_ALGO_DEMO,
}basic_algo_type_t;

/** algorithm request cmd for sensor_core*/
enum ipc_cmd_algo2core_type {
	CMD_SET_HW_TAPPING_AS_WAKEUP_SOURCE,
	CMD_ALGO2CORE_MAX,
};

/**
 * exposed sensor structure used for interfacing with sensor service
 **/
typedef struct {
	list_t link;
	uint8_t type;
	uint8_t id;
	uint32_t depend_flag;
	uint8_t ready_flag;
	void *rpt_data_buf;
	uint16_t rpt_data_buf_len;
	uint16_t stat_flag;
	uint8_t data_frame_count;
}exposed_sensor_t;

/**
 * raw data demand structure used for define kinds of raw data in feed
 **/
typedef struct {
	uint8_t type;
	uint8_t id;
	uint8_t flag;
	int16_t range_max;
	int16_t range_min;
	int freq;
	float rt;
	void *match_buffer;
	uint16_t match_buffer_repo;
	uint16_t match_data_count;
	uint16_t put_idx;
	uint16_t get_idx;
	int16_t raw_data_offset;
}sensor_data_demand_t;

/**
 * feed control api structure, which are called by open sensor core
 **/
struct feed_general_t;
typedef struct {
	int (*exec)(void **, struct feed_general_t *);
	int (*init)(struct feed_general_t *);
	int (*deinit)(struct feed_general_t *);
	int (*reset)(struct feed_general_t *);
	int (*goto_idle)(struct feed_general_t *);
	int (*out_idle)(struct feed_general_t *);
	int (*get_property)(struct feed_general_t *, uint8_t exposed_type,
			    uint8_t exposed_id, uint8_t property_type,
			    void *ptr_return);
	int (*set_property)(struct feed_general_t *, uint8_t exposed_type,
			    uint8_t exposed_id, uint8_t param_length,
			    void *ptr_param);
}feed_control_api_t;

/**
 * feed structure, which is agent of algo
 * and used by open sensor core to manage kinds of algos
 **/
typedef struct feed_general_t {
	list_t link;
	sensor_data_demand_t *demand;
	uint32_t report_flag;
	uint16_t stat_flag;
	uint16_t mark_flag;
	feed_control_api_t ctl_api;
	void *param;
	int8_t demand_length;
	uint8_t rf_cnt;
	basic_algo_type_t type;
	uint8_t idle_hold_flag : 1;
	uint8_t no_idle_flag : 1;
	uint8_t wake_up_clear_fifo_flag : 1;
	uint8_t motion_sensor_flag : 1;
}feed_general_t;

/** @} */
#endif
