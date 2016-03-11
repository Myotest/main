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
/* *INDENT-OFF* */
#include "opencore_main.h"

static char commit_buf[sizeof(struct ia_cmd)
		+ sizeof(struct sensor_data)
		+ COMMIT_DATA_MAX_LENGTH]__attribute__((section(".dccm")));
static uint16_t algo_engine_port;
extern uint8_t read_out_fifo_flag;

void OpencoreCommitSensData(uint8_t type, uint8_t id, void* data_ptr, int data_length)
{
	exposed_sensor_t* report_sensor = GetExposedStruct(type, id);
	if(report_sensor != NULL && (report_sensor->stat_flag & SUBSCRIBED) != 0
		&& data_ptr != NULL && data_length != 0){
		struct ia_cmd* cmd;
		struct sensor_data* data;
		memset(&commit_buf[0], 0, sizeof(commit_buf));
		cmd = (struct ia_cmd*)&commit_buf[0];
		cmd->cmd_id = SENSOR_DATA;
		cmd->length = sizeof(struct ia_cmd) + sizeof(struct sensor_data) + data_length;
		data = (struct sensor_data*)(cmd + 1);
		data->sensor.sensor_type = report_sensor->type;
		data->sensor.dev_id = report_sensor->id;
		data->timestamp = get_uptime_ms();
		data->data_length = data_length;
		if( 0 < data_length && data_length<= 60){
			memcpy(data->data, data_ptr, data_length);
			ipc_2svc_send(cmd);
		}
	}
}

static void HandleAlgo(feed_general_t* feed, void** data_ptr)
{
	int ret = feed->ctl_api.exec(data_ptr, feed);
	if(ret != 0){
		for(list_t* next = exposed_sensor_list.head; next != NULL; next = next->next){
			exposed_sensor_t* exposed_sensor = (exposed_sensor_t*)next;
			if(exposed_sensor->ready_flag != 0){
				OpencoreCommitSensData(exposed_sensor->type, exposed_sensor->id,
					exposed_sensor->rpt_data_buf, exposed_sensor->rpt_data_buf_len);
				exposed_sensor->ready_flag = 0;
			}
		}
	}
}

static void AddCaliData(uint8_t phy_type, sensor_handle_t* phy_sensor, void* ptr_from)
{
	switch(phy_type){
		case SENSOR_ACCELEROMETER:
			{
				short* cali_ptr = (short*)phy_sensor->clb_data_buffer;
				for(int k = 0; k < 3; k++)
					((short*)ptr_from)[k] += cali_ptr[k];
			}
			break;
		case SENSOR_GYROSCOPE:
		case SENSOR_MAGNETOMETER:
			{
				int* cali_ptr = (int*)phy_sensor->clb_data_buffer;
				for(int k = 0; k < 3; k++)
					((int*)ptr_from)[k] += cali_ptr[k];
			}
			break;
		default:
			break;
	}
}

static void HandleMatchBufferData(feed_general_t* feed)
{
	sensor_data_demand_t* demand = feed->demand;
	uint8_t demand_length = feed->demand_length;
	uint8_t data_match_not_ready_flag = 0;
	uint16_t cm_time_consume = 1;
	uint8_t match_times = 0;

	for(int i = 0; i < demand_length; i++){
		if(demand[i].freq == 0)
			continue;
		if((demand[i].flag & IGNORE) != 0)
			continue;
		cm_time_consume = GetCommonMultiple(cm_time_consume, ValueRound((float)1000 / demand[i].freq));
	}

	for(int i = 0; i < demand_length; i++){
		if(demand[i].freq == 0)
			continue;
		if((demand[i].flag & IGNORE) != 0)
			continue;
		if(demand[i].match_data_count < cm_time_consume / ValueRound((float)1000 / demand[i].freq)){
			data_match_not_ready_flag++;
		}else{
			int temp = demand[i].match_data_count / (cm_time_consume / ValueRound((float)1000 / demand[i].freq));
			if(match_times == 0 || temp < match_times)
				match_times = temp;
		}
	}

	if(data_match_not_ready_flag == 0){
		int vernier_length = 0;
		uint16_t cm_multi_freq = 1;
		int8_t scale[demand_length];
		memset(scale, 0, sizeof(scale));
		for(int i = 0; i < demand_length; i++){
			if(demand[i].freq == 0)
				continue;
			cm_multi_freq = GetCommonMultiple(cm_multi_freq, demand[i].freq);
		}

		for(int i = 0; i < demand_length; i++){
			int count;
			if(demand[i].freq == 0)
				continue;
			scale[i] = cm_multi_freq / demand[i].freq;
			count  = match_times * (cm_time_consume / ValueRound((float)1000 / demand[i].freq));
			if(vernier_length == 0 || vernier_length < count * scale[i])
				vernier_length = count * scale[i];
		}

		for(int v = 0; v < vernier_length; v++){
			void* ptr[demand_length];
			memset(ptr, 0, sizeof(ptr));
			int act = 0;
			for(int i = 0; i < demand_length; i++){
				if(demand[i].freq == 0)
					continue;
				sensor_handle_t* phy_sensor = GetActivePollSensStruct(demand[i].type, demand[i].id);
				if(phy_sensor != NULL && scale[i] != 0 && demand[i].match_buffer != NULL){
					if(v % scale[i] == 0 && demand[i].get_idx != demand[i].put_idx){
						void* ptr_from = demand[i].match_buffer
							+ phy_sensor->sensor_data_frame_size * demand[i].get_idx;
						//add the calibration offset value
						AddCaliData(demand[i].type, phy_sensor, ptr_from);
						ptr[i] = ptr_from;
						demand[i].get_idx++;
						demand[i].match_data_count--;
						if(demand[i].get_idx >= demand[i].match_buffer_repo)
							demand[i].get_idx = 0;
						act++;
					}
				}
			}

			if(act != 0 && feed->ctl_api.exec != NULL)
				HandleAlgo(feed, ptr);
		}
	}
}

static void FeedSensDataDirectly(feed_general_t* feed)
{
	sensor_data_demand_t* demand = feed->demand;
	uint8_t demand_length = feed->demand_length;
	for(int i = 0; i < demand_length; i++){
		if(demand[i].freq == 0)
			continue;
		sensor_handle_t* phy_sensor = GetActivePollSensStruct(demand[i].type, demand[i].id);
		if(phy_sensor != NULL){
			list_t* node = phy_sensor->raw_data_head[phy_sensor->head_for_algo].head;
			while(node != NULL){
				//get raw data node
				raw_data_node_t* raw_data = (raw_data_node_t*)((void*)node
					- offsetof(raw_data_node_t, raw_data_node));
				uint16_t raw_sensor_data_count = raw_data->raw_data_count;
				void* buffer = raw_data->buffer;
				int gap = ValueRound((float)phy_sensor->freq / (demand[i].freq * 10));
				int frame_size = phy_sensor->sensor_data_frame_size;
				int count;
				void* ptr[demand_length];
				memset(ptr, 0, sizeof(ptr));

				for(count = 0; gap * count + demand[i].raw_data_offset < raw_sensor_data_count; count++){
					int idx = gap * count + demand[i].raw_data_offset;
					void* ptr_from = phy_sensor->feed_data_buffer;
					memcpy(ptr_from, buffer + idx * frame_size, frame_size);
					//add cali offset
					AddCaliData(demand[i].type, phy_sensor, ptr_from);
					ptr[i] = ptr_from;
					if(feed->ctl_api.exec != NULL)
						HandleAlgo(feed, ptr);
				}
				demand[i].raw_data_offset = gap - (raw_sensor_data_count
					- (gap * (count - 1) + demand[i].raw_data_offset));
				node = node->next;
			}
		}
	}
}

static void CopySensorData2DelayBuf(sensor_data_demand_t* demand, void* buffer,
				int gap, int count, int frame_size)
{
	int idx = gap * count + demand->raw_data_offset;
	if(demand->match_buffer != NULL){
		memcpy(demand->match_buffer + demand->put_idx * frame_size,
			buffer + idx * frame_size, frame_size);
		demand->put_idx++;
		demand->match_data_count++;
		if(demand->put_idx == demand->match_buffer_repo)
			demand->put_idx = 0;
		if(demand->put_idx == demand->get_idx){
			demand->get_idx++;
			if(demand->get_idx == demand->match_buffer_repo)
				demand->get_idx = 0;
				demand->match_data_count--;
		}
	}
}

static void FeedSensDataAfterMatch(feed_general_t* feed, match_t type)
{
	sensor_data_demand_t* demand = feed->demand;
	uint8_t demand_length = feed->demand_length;
	list_t* node[demand_length];
	memset(node, 0, sizeof(node));
	int d_valid_cnt = 0;
	uint32_t cm_time_consume = 1;
	int count[demand_length];
	memset(count, 0, sizeof(count));

	for(int i = 0; i < demand_length; i++){
		if(demand[i].freq == 0)
			continue;
		cm_time_consume = GetCommonMultiple(cm_time_consume,
			ValueRound((float)1000 / demand[i].freq));
	}

	for(int i = 0; i < demand_length; i++){
		if(demand[i].freq == 0)
			continue;
		sensor_handle_t* phy_sensor = GetActivePollSensStruct(demand[i].type, demand[i].id);
		if(phy_sensor != NULL)
			//get raw data node
			node[i] = phy_sensor->raw_data_head[phy_sensor->head_for_algo].head;
		d_valid_cnt++;
	}

	while(1){
		int defect = 0;
		for(int i = 0; i < demand_length; i++){
			if(demand[i].freq == 0)
				continue;
			sensor_handle_t* phy_sensor = GetActivePollSensStruct(demand[i].type, demand[i].id);
			if ((phy_sensor != NULL) && (node[i] != NULL)) {
				raw_data_node_t* raw_data = (raw_data_node_t*)((void*)node[i] - offsetof(raw_data_node_t, raw_data_node));
				uint16_t raw_sensor_data_count = raw_data->raw_data_count;
				void* buffer = raw_data->buffer;
				int gap = ValueRound((float)phy_sensor->freq / (demand[i].freq * 10));

				if(type == SYNC){
					int	target_count = cm_time_consume / ValueRound((float)1000 / demand[i].freq);
					for(; gap * count[i] + demand[i].raw_data_offset < raw_sensor_data_count
							&& demand[i].match_data_count < target_count; count[i]++){
						CopySensorData2DelayBuf(&demand[i], buffer, gap, count[i], phy_sensor->sensor_data_frame_size);
					}
					if(gap * count[i] + demand[i].raw_data_offset >= raw_sensor_data_count){
						demand[i].raw_data_offset = gap - (raw_sensor_data_count
							- (gap * (count[i] - 1) + demand[i].raw_data_offset));
						count[i] = 0;
						node[i] = node[i]->next;
					}
				}else if(type == ASYNC){
					int count;
					for(count = 0; gap * count + demand[i].raw_data_offset
						< raw_sensor_data_count; count++){
						CopySensorData2DelayBuf(&demand[i], buffer, gap, count, phy_sensor->sensor_data_frame_size);
					}

					demand[i].raw_data_offset = gap - (raw_sensor_data_count
						- (gap * (count - 1) + demand[i].raw_data_offset));
					node[i] = node[i]->next;
				}
			}else
				defect++;
		}

		if(defect == d_valid_cnt && type == ASYNC)
			break;
		if(defect > 0 && type == SYNC)
			break;

		HandleMatchBufferData(feed);
	}
}

static void FeedSensData2Algo(void)
{
	for(list_t* next = phy_sensor_poll_active_list.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next
			- offsetof(sensor_handle_t, links.poll.poll_active_link));
		uint8_t no = phy_sensor->head_for_algo == 0 ? 1 : 0;
		uint32_t key = irq_lock();
		phy_sensor->head_for_algo = no;
		irq_unlock(key);
	}

	for(list_t* next = feed_list.head; next != NULL; next = next->next){
		feed_general_t* feed = (feed_general_t*)next;
		if((feed->stat_flag & ON) != 0 && ((feed->stat_flag & IDLE) == 0 || (feed->stat_flag & WAKE_UP_CLEAR_FIFO) != 0)){
			//classify feed and do with it
			if(feed->type != BASIC_ALGO_RAWDATA && feed->demand_length == 1){
				FeedSensDataDirectly(feed);
				continue;
			}
			if(feed->type == BASIC_ALGO_RAWDATA){
				int active_count = 0;
				for(int i = 0; i < feed->demand_length; i++)
					if(feed->demand[i].freq != 0)
						active_count++;

				if(active_count == 1){
					FeedSensDataDirectly(feed);
					continue;
				}
			}

			int ret = CheckMinDelayBuffer(feed);
			if(ret == 0){
				FeedSensDataAfterMatch(feed, SYNC);
				continue;
			}
			FeedSensDataAfterMatch(feed, ASYNC);
		}
	}

	for(list_t* next = phy_sensor_poll_active_list.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next
				- offsetof(sensor_handle_t, links.poll.poll_active_link));
		list_t* node = phy_sensor->raw_data_head[phy_sensor->head_for_algo].head;
		while(node != NULL){
			list_t* temp = node->next;
			raw_data_node_t* raw_data = (raw_data_node_t*)((void*)node
				- offsetof(raw_data_node_t, raw_data_node));
			if(phy_sensor->fifo_length == 0 || phy_sensor->fifo_use_flag == 0)
				FreeInDss(raw_data->buffer);
			FreeInDss((void*)node);
			node = temp;
		}
		memset(&phy_sensor->raw_data_head[phy_sensor->head_for_algo], 0, sizeof(list_head_t));
		phy_sensor->fifo_share_read_sync_done = 0;
	}
}

static int ReadFifo(sensor_handle_t* phy_sensor, int* fifo_clear)
{
	int ret = 0;
	list_t* share_list_head = &phy_sensor->fifo_share_link;
	int phy_type_fifo_clear_cnt = 0, phy_type_active_cnt = 0;

	do{
		sensor_handle_t* phy_sensor_node = (sensor_handle_t*)((void*)share_list_head
			- offsetof(sensor_handle_t, fifo_share_link));
		if((phy_sensor_node->stat_flag & ON) != 0 && phy_sensor_node->fifo_share_read_sync_done == 0
			&& ((phy_sensor_node->stat_flag & IDLE) == 0 || read_out_fifo_flag != 0)){
			mutex_lock(phy_sensor_node->mutex, OS_WAIT_FOREVER);
			ret = phy_sensor_fifo_read(phy_sensor_node->ptr, (uint8_t*)phy_sensor_node->buffer,
				(uint16_t)phy_sensor_node->buffer_length);

			if(ret < phy_sensor->buffer_length)
				phy_type_fifo_clear_cnt++;
			phy_type_active_cnt++;

			if(ret != 0){
				raw_data_node_t* node = (raw_data_node_t*)AllocFromDss(sizeof(raw_data_node_t));
				if(node == NULL)
					pr_error(LOG_MODULE_OPEN_CORE, "fail to alloc raw data node fifo type=%d", phy_sensor_node->type);
				if(node != NULL){
					node->buffer = phy_sensor_node->buffer;
					node->raw_data_count = ret / phy_sensor_node->sensor_data_frame_size;
					uint8_t head_for_raw = phy_sensor_node->head_for_algo == 0 ? 1 : 0;
					list_add(&phy_sensor_node->raw_data_head[head_for_raw], &node->raw_data_node);
				}else{
					if(node != NULL)
						FreeInDss((void*)node);
				}
				phy_sensor_node->fifo_share_read_sync_done = 1;
			}
			mutex_unlock(phy_sensor_node->mutex);
		}
		share_list_head = share_list_head->next;
	}while(share_list_head != &phy_sensor->fifo_share_link);

	if(fifo_clear != NULL && phy_type_fifo_clear_cnt < phy_type_active_cnt)
		*fifo_clear = FIFO_NOT_CLEAR;

	return ret;
}

static void handle_algo_ref_port(struct message* m, void* data)
{
	switch(MESSAGE_ID(m)){
		case ALGO_PROCESS:
			FeedSensData2Algo();
			break;
		case ALGO_GOTO_IDLE:{
				algo_engine_msg_t* msg = (algo_engine_msg_t*)m;
				feed_general_t* feed = (feed_general_t*)msg->priv;
				if(feed->ctl_api.goto_idle != NULL)
					feed->ctl_api.goto_idle(feed);
			}
			break;
		case ALGO_OUT_IDLE:{
				algo_engine_msg_t* msg = (algo_engine_msg_t*)m;
				feed_general_t* feed = (feed_general_t*)msg->priv;
				if(feed->ctl_api.out_idle != NULL)
					feed->ctl_api.out_idle(feed);
			}
			break;
		case READ_FIFO:{
				algo_engine_msg_t* msg = (algo_engine_msg_t*)m;
				sensor_handle_t* phy_sensor = (sensor_handle_t*)msg->priv;
				if(phy_sensor->fifo_length > 0 && phy_sensor->fifo_use_flag != 0)
					ReadFifo(phy_sensor, NULL);
			}
			break;
		case CLEAR_FIFOS:{
				int phy_type_fifo_clean_cnt = 0, phy_type_fifo_cnt = 0;
				for(list_t* node = phy_sensor_poll_active_list.head; node != NULL; node = node->next){
					sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)node
						- offsetof(sensor_handle_t, links.poll.poll_active_link));
					int fifo_clear = 0;
					if(phy_sensor->fifo_length > 0 && phy_sensor->fifo_use_flag != 0 && phy_sensor->buffer != NULL){
						ReadFifo(phy_sensor, &fifo_clear);
						if(fifo_clear != FIFO_NOT_CLEAR)
							phy_type_fifo_clean_cnt++;
						phy_type_fifo_cnt++;
					}
				}

				if(phy_type_fifo_clean_cnt == phy_type_fifo_cnt)
					read_out_fifo_flag = 0;

				FeedSensData2Algo();

				struct ia_cmd* cmd = (struct ia_cmd*)balloc(sizeof(struct ia_cmd), NULL);
				cmd->cmd_id = CMD_FIFO_CLEAR_CHECK_SC;
				ipc_2core_send(cmd);
			}
			break;
		default:
			break;
	}
	bfree(m);
}

void TriggerAlgoEngine(uint16_t msg_id, void* priv_data)
{
	algo_engine_msg_t* algo_msg = (algo_engine_msg_t*)message_alloc(sizeof(algo_engine_msg_t), NULL);
	algo_msg->m.id = msg_id;
	algo_msg->m.dst_port_id = algo_engine_port;
	algo_msg->priv = priv_data;
	port_send_message(&algo_msg->m);
}

void AlgoEngineInit(T_QUEUE service_mgr_queue)
{
	algo_engine_port = port_alloc(service_mgr_queue);
	port_set_handler(algo_engine_port, handle_algo_ref_port, NULL);
}
/* *INDENT-ON* */
