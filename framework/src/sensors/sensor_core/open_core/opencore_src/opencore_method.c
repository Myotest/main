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
#include "opencore_main.h"
sensor_handle_t *GetIntSensStruct(uint8_t type, uint8_t id)
{
	for (list_t *next = phy_sensor_list_int.head;
	     next != NULL;
	     next = next->next) {
		sensor_handle_t *phy_sensor =
			(sensor_handle_t *)((void *)next -
					    offsetof(sensor_handle_t,
						     links.int_link));
		if ((id == DEFAULT_ID && phy_sensor->type == type)
		    || (phy_sensor->type == type && phy_sensor->id == id))
			return phy_sensor;
	}
	return NULL;
}

feed_general_t *GetFeedStruct(basic_algo_type_t type)
{
	for (list_t *next = feed_list.head; next != NULL; next = next->next)
		if (((feed_general_t *)next)->type == type)
			return (feed_general_t *)next;
	return NULL;
}

exposed_sensor_t *GetExposedStruct(uint8_t type, uint8_t id)
{
	for (list_t *next = exposed_sensor_list.head;
	     next != NULL;
	     next = next->next) {
		exposed_sensor_t *exposed_sensor = (exposed_sensor_t *)next;
		if (exposed_sensor->type == type && exposed_sensor->id == id)
			return exposed_sensor;
	}
	return NULL;
}

int GetHashKey(uint8_t phy_sensor_type, uint8_t dev_id)
{
	return (phy_sensor_type % PHY_TYPE_KEY_LENGTH) * PHY_ID_KEY_LENGTH
	       + dev_id % PHY_ID_KEY_LENGTH;
}

sensor_handle_t *GetActivePollSensStruct(int8_t type, uint8_t id)
{
	int idx = GetHashKey(type, id);

	for (list_t *next = phy_sensor_poll_active_array[idx].head;
	     next != NULL;
	     next = next->next) {
		sensor_handle_t *phy_sensor =
			(sensor_handle_t *)((void *)next -
					    offsetof(sensor_handle_t,
						     links.poll.
						     poll_active_array_link));
		if (phy_sensor->type == type && phy_sensor->id == id)
			return phy_sensor;
	}
	return NULL;
}

sensor_handle_t *GetPollSensStruct(uint8_t type, uint8_t id)
{
	for (list_t *next = phy_sensor_list_poll.head;
	     next != NULL;
	     next = next->next) {
		sensor_handle_t *phy_sensor =
			(sensor_handle_t *)((void *)next -
					    offsetof(sensor_handle_t,
						     links.poll.poll_link));
		if (phy_sensor->type == type && phy_sensor->id == id)
			return phy_sensor;
	}
	return NULL;
}

int GetPhyRegData(uint8_t type, uint8_t dev_id, void *buffer, int buf_len)
{
	int mark = 0, ret = 0;
	sensor_handle_t *phy_sensor = GetPollSensStruct(type, dev_id);

	if (phy_sensor == NULL)
		return -1;

	if ((phy_sensor->stat_flag & ON) == 0) {
		int length = sizeof(struct sensor_data) +
			     phy_sensor->sensor_data_frame_size;
		struct sensor_data *temp_ptr = NULL;
		temp_ptr = (struct sensor_data *)balloc(length, NULL);
		temp_ptr->data_length = phy_sensor->sensor_data_frame_size;
		phy_sensor->buffer = (void *)temp_ptr->data;
		phy_sensor_enable(phy_sensor->ptr, 1);
		mark++;
	}

	void *temp_ptr = phy_sensor->buffer - offsetof(struct sensor_data, data);
	if (0 ==
	    phy_sensor_data_read(phy_sensor->ptr,
				 (struct sensor_data *)temp_ptr)) {
		ret = -1;
		goto out;
	}

	memcpy(buffer, phy_sensor->buffer, buf_len);
out:
	if (mark != 0)
		phy_sensor_enable(phy_sensor->ptr, 0);
	return ret;
}

int GetSensorDataFrameSize(uint8_t type, uint8_t id)
{
	sensor_handle_t *phy_sensor = GetActivePollSensStruct(type, id);

	if (phy_sensor != NULL) {
		return phy_sensor->sensor_data_frame_size;
	}
	return -1;
}

uint16_t GetSensSamplingTime(sensor_handle_t *phy_sensor)
{
	uint16_t odr = 0;

	if (phy_sensor != NULL)
		phy_sensor_get_odr_value(phy_sensor->ptr, &odr);
	return odr;
}

uint16_t SetSensSamplingTime(sensor_handle_t *phy_sensor, uint16_t odr_x10)
{
	int ret = 0;

	if (phy_sensor != NULL) {
		ret = phy_sensor_set_odr_value(phy_sensor->ptr, odr_x10);
		if (ret != 0)
			phy_sensor->freq = odr_x10;
	}
	return ret;
}

static uint16_t GetCommonDivisor(uint16_t next_freq, uint16_t prev_multiple)
{
	uint16_t temp;

	if (next_freq < prev_multiple) {
		temp = next_freq;
		next_freq = prev_multiple;
		prev_multiple = temp;
	}
	temp = next_freq % prev_multiple;
	while (temp != 0) {
		next_freq = prev_multiple;
		prev_multiple = temp;
		temp = next_freq % prev_multiple;
	}
	return prev_multiple;
}

uint16_t GetCommonMultiple(uint16_t next_freq, uint16_t prev_multiple)
{
	return next_freq * prev_multiple / GetCommonDivisor(next_freq,
							    prev_multiple);
}

int ValueRound(float value)
{
	int rslt = value;
	int temp = ((int)(value * 10)) % 10;

	if (temp >= 5)
		rslt++;
	return rslt;
}

int SendCmd2OpenCore(int cmd_id)
{
	int length = sizeof(struct ia_cmd);
	struct ia_cmd *cmd = (struct ia_cmd *)balloc(length, NULL);

	if (cmd == NULL)
		return -1;

	memset(cmd, 0, length);
	cmd->cmd_id = cmd_id;
	ipc_2core_send(cmd);
	return 0;
}
