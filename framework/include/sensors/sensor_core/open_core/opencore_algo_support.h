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
#ifndef __OPENCORE_ALGO_SUPPORT_H__
#define __OPENCORE_ALGO_SUPPORT_H__
/**
 * @addtogroup open_sensor_core
 * @{
 */
#include "opencore_algo_common.h"

extern exposed_sensor_t *GetExposedStruct(uint8_t type, uint8_t id);
extern void OpencoreCommitSensData(uint8_t type, uint8_t id, void *data_ptr,
				   int data_length);
extern int GetSensorDataFrameSize(uint8_t type, uint8_t id);
extern int GetPhyRegData(uint8_t type, uint8_t dev_id, void *buffer,
			 int buf_len);
extern int SendCmd2OpenCore(int cmd_id);

/**
 * this macro is used to assign the pointer of feed struct defined
 * by user into feed section.
 **/
#define define_feedinit(name) \
	static void *__feedinit_ ## name __used	\
	__section(".openinit.feed") = &name
extern char _s_feedinit[], _e_feedinit[];

/**
 * this macro is used to assign the pointer of exposed sensor struct
 * defined by user into exposed section.
 **/
#define define_exposedinit(name) \
	static void *__exposedinit_ ## name __used \
	__section(".openinit.exposed") = &name
extern char _s_exposedinit[], _e_exposedinit[];

#define HOLD_WAKE_UP_CLEAR_FIFO(feed) do {	 \
		feed->wake_up_clear_fifo_flag = 1;	\
} while (0);

/**
 * this macro is used to stop going to idle state, which means keep fed raw sensor data
 * even though no motion occurs.
 **/
#define STOP_IDLE(feed) do {	 \
		feed->idle_hold_flag = 1;	\
} while (0);

/**
 * this macro is used to start go to idle state, which means no need for more sensor data
 * while no motion.
 **/
#define START_IDLE(feed) do { \
		feed->idle_hold_flag = 0; \
} while (0);

/**
 * this macro is used to keep fed raw sensor data all the time.
 **/
#define HOLD_IDLE(feed) do {	 \
		feed->no_idle_flag = 1;		\
} while (0);

/**
 *  @brief  Get the index of physical sensor type in the demand array of feed.
 * @param[in]  feed : the feed structure
 * @param[in]  type : the physical sensor type
 * @return the demand index in feed that defined by user.
 */
static inline int GetDemandIdx(feed_general_t *feed, uint8_t type)
{
	int i;

	for (i = 0; i < feed->demand_length; i++)
		if (feed->demand[i].type == type)
			return i;
	return -1;
}

/**
 *  @brief  With this api, algorithm can report algo exec result to app directly
 * @param[in]  exposed_sensor : pointer of exposed sensor
 */
static inline void ReportSensDataDirectly(exposed_sensor_t *exposed_sensor)
{
	OpencoreCommitSensData(exposed_sensor->type, exposed_sensor->id,
			       exposed_sensor->rpt_data_buf,
			       exposed_sensor->rpt_data_buf_len);
	exposed_sensor->ready_flag = 0;
}
/** @} */
#endif
