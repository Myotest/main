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
#include "opencore_support.h"
list_head_t feed_list;
list_head_t exposed_sensor_list;

list_head_t phy_sensor_list_int;
list_head_t phy_sensor_list_poll;
list_head_t phy_sensor_poll_active_list;
list_head_t phy_sensor_poll_active_array[PHY_TYPE_KEY_LENGTH * PHY_ID_KEY_LENGTH];

struct pm_wakelock opencore_main_wl;
struct pm_wakelock opencore_cali_wl;
uint8_t global_suspend_flag = 1;
uint8_t raw_data_dump_flag = 0;

#ifdef SUPPORT_INTERRUPT_MODE
static void raw_data_fifo_int_cb(phy_sensor_event_t* event, void* priv_data);
#endif

#define BUF_CNT_1024 1
#define BUF_CNT_512 2
#define BUF_CNT_128 2
#define BUF_CNT_64  12

static uint8_t buffer_for_raw_sensor_data[64*BUF_CNT_64
	+ 128*BUF_CNT_128
	+ 512*BUF_CNT_512
	+ 1024*BUF_CNT_1024] __attribute__((section(".dccm")));
static volatile uint32_t buf_mask_64;
static volatile uint8_t buf_mask_128;
static volatile uint8_t buf_mask_256;
static volatile uint8_t buf_mask_512;
static volatile uint8_t buf_mask_1024;

void* AllocFromDss(uint32_t size)
{
	void* ptr = NULL;
	void* start = (void*)&buffer_for_raw_sensor_data[0];
	uint32_t key = irq_lock();
	if(size <= 64 && buf_mask_64 != (1 << BUF_CNT_64) - 1){
		for(int i = 0; i < BUF_CNT_64; i++){
			if((buf_mask_64 & (1 << i)) == 0){
				ptr = start + 64*i;
				buf_mask_64 |= 1 << i;
				break;
			}
		}
	}else if(size <= 128 && buf_mask_128 != (1 << BUF_CNT_128) - 1){
		for(int i = 0; i < BUF_CNT_128; i++){
			if((buf_mask_128 & (1 << i)) == 0){
				ptr = start + 64*BUF_CNT_64 + 128*i;
				buf_mask_128 |= 1 << i;
				break;
			}
		}
	}else if(size <= 512 && buf_mask_512 != (1 << BUF_CNT_512) - 1){
		for(int i = 0; i < BUF_CNT_512; i++){
			if((buf_mask_512 & (1 << i)) == 0){
				ptr = start + 64*BUF_CNT_64 + 128*BUF_CNT_128 + 512*i;
				buf_mask_512 |= 1 << i;
				break;
			}
		}
	}else if(size <= 1024 && buf_mask_1024 != (1 << BUF_CNT_1024) - 1){
		for(int i = 0; i < BUF_CNT_1024; i++){
			if((buf_mask_1024 & (1 << i)) == 0){
				ptr = start + 64*BUF_CNT_64 + 128*BUF_CNT_128 + 512*BUF_CNT_512 + 1024*i;
				buf_mask_1024 |= 1 << i;
				break;
			}
		}
	}

	irq_unlock(key);
	return ptr;
}

int FreeInDss(void* buf)
{
	int ret = -1;
	void* start = (void*)&buffer_for_raw_sensor_data[0];
	uint32_t key = irq_lock();
	if(buf < start + 64*BUF_CNT_64){
		for(int i = 0; i < BUF_CNT_64; i++){
			if(buf == start + 64*i){
				buf_mask_64 &= ~(1<<i);
				ret = 0;
				break;
			}
		}
	}else if(buf < start + 64*BUF_CNT_64 + 128*BUF_CNT_128){
		for(int i = 0; i < BUF_CNT_128; i++){
			if(buf == start + 64*BUF_CNT_64 + 128*i){
				buf_mask_128 &= ~(1<<i);
				ret = 0;
				break;
			}
		}
	}else if(buf < start + 64*BUF_CNT_64 + 128*BUF_CNT_128 + 512*BUF_CNT_512){
		for(int i = 0; i < BUF_CNT_512; i++){
			if(buf == start + 64*BUF_CNT_64 + 128*BUF_CNT_128 + 512*i){
				buf_mask_512 &= ~(1<<i);
				ret = 0;
				break;
			}
		}
	}else if(buf < start + 64*BUF_CNT_64 + 128*BUF_CNT_128 + 512*BUF_CNT_512 + 1024*BUF_CNT_1024){
		for(int i = 0; i < BUF_CNT_1024; i++){
			if(buf == start + 64*BUF_CNT_64 + 128*BUF_CNT_128 + 512*BUF_CNT_512 + 1024*i){
				buf_mask_1024 &= ~(1<<i);
				ret = 0;
				break;
			}
		}
	}

	irq_unlock(key);
	return ret;
}

static uint16_t MatchFreq(sensor_handle_t* phy_sensor, uint16_t freq)
{
	uint16_t final_freq = 1;
	if(phy_sensor != NULL){
		int min_gap = 0, final_gap = 0;
		int up_gap, down_gap;
		int gap_flag;
		uint16_t ret_freq;
		uint16_t min_freq = 0;

		phy_sensor_query_odr_value(phy_sensor->ptr, 1, &min_freq);

		for(int i = 1; freq * i <= 10000; i++){
			gap_flag = 0;
			phy_sensor_query_odr_value(phy_sensor->ptr, freq * i, &ret_freq);

			if(ret_freq == min_freq)
				min_gap = ret_freq - freq * i;
			else{
				up_gap = ret_freq - freq * i;
				down_gap = freq * i - ret_freq / 2;
				if(up_gap >= down_gap){
					gap_flag = 1;
					min_gap = down_gap;
				}else
					min_gap = up_gap;
			}

			if(final_gap > min_gap || i == 1){
				final_gap = min_gap;
				if(gap_flag == 1)
					final_freq = ret_freq / 2;
				else
					final_freq = ret_freq;
			}
		}
	}
	return final_freq;
}

static void ClosePhySensor(sensor_handle_t* phy_sensor)
{
	if(phy_sensor->fifo_length > 0)
		mutex_lock(phy_sensor->mutex, OS_WAIT_FOREVER);

	if(phy_sensor->buffer_length / phy_sensor->sensor_data_frame_size > 1){
		phy_sensor_enable_hwfifo(phy_sensor->ptr, 0, 0);
		if(phy_sensor->buffer != NULL)
			FreeInDss(phy_sensor->buffer);
	}else{
		phy_sensor_enable(phy_sensor->ptr, 0);
		if(phy_sensor->buffer != NULL){
			void* temp_ptr = phy_sensor->buffer - offsetof(struct sensor_data, data);
			bfree(temp_ptr);
		}
	}
	for(int i = 0; i < RAW_DATA_HEAD_CNT; i++){
		list_t* node = phy_sensor->raw_data_head[i].head;
		while(node != NULL){
			list_t* temp = node->next;
			raw_data_node_t* raw_data = (raw_data_node_t*)((void*)node
				- offsetof(raw_data_node_t, raw_data_node));
			FreeInDss(raw_data->buffer);
			FreeInDss((void*)node);
			node = temp;
		}
		memset(&phy_sensor->raw_data_head[i], 0, sizeof(list_head_t));
	}
	SetSensSamplingTime(phy_sensor, 0);
	phy_sensor->buffer = NULL;
	phy_sensor->buffer_length = 0;
	phy_sensor->npp = 0;
	if(phy_sensor->fifo_length > 0)
		mutex_unlock(phy_sensor->mutex);
}

void CloseIntSensor(sensor_handle_t* phy_sensor)
{
	phy_sensor_data_unregister_callback(phy_sensor->ptr);
	phy_sensor_enable(phy_sensor->ptr, 0);
	phy_sensor->stat_flag &= ~ON;
}

void OpenIntSensor(sensor_handle_t* phy_sensor)
{
	phy_sensor_enable(phy_sensor->ptr, 1);
	phy_sensor_data_register_callback(phy_sensor->ptr, &motion_detect_callback, NULL, 0);
	phy_sensor->stat_flag |= ON;
}

static void ResetPhySensorList(void)
{
	for(list_t* next = phy_sensor_poll_active_list.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next
			- offsetof(sensor_handle_t, links.poll.poll_active_link));
		phy_sensor->stat_flag &= ~ON;
		phy_sensor->idle_ref = 0;
		phy_sensor->pi = phy_sensor->npp = 0;
		phy_sensor->freq = 0;
		phy_sensor->fifo_use_flag = phy_sensor->fifo_share_done_flag = 0;
		phy_sensor->dirty = 1;
	}
	phy_sensor_poll_active_list.head = phy_sensor_poll_active_list.tail = NULL;
	memset(phy_sensor_poll_active_array, 0, sizeof(phy_sensor_poll_active_array));
}

static void ResetDemandDelayBuffer(sensor_data_demand_t* demand, int count, sensor_handle_t* phy_sensor)
{
	int length;
	if(count < 0){
		if(demand->match_buffer_repo == 0)
			return;
		count = demand->match_buffer_repo;
	}

	if(demand->match_buffer != NULL){
		bfree(demand->match_buffer);
		demand->match_buffer = NULL;
	}
	length = count * phy_sensor->sensor_data_frame_size;
	demand->match_buffer = balloc(length, NULL);
	if(demand->match_buffer == NULL)
		return;

	memset(demand->match_buffer, 0, length);
	demand->match_buffer_repo = count;

	demand->put_idx = demand->get_idx = 0;
	demand->match_data_count = 0;
}

int CheckMinDelayBuffer(feed_general_t* feed)
{
	int ret = -1;
	uint8_t ko_flag = 0;
	float equal_pi = 0;
	uint8_t act = 0;
	sensor_data_demand_t* demand = feed->demand;
	uint8_t demand_length = feed->demand_length;

	for(int i = 0; i < demand_length; i++){
		if(demand[i].freq == 0)
			continue;
		if((demand[i].flag & IGNORE) != 0)
			continue;
		sensor_handle_t* phy_sensor = GetActivePollSensStruct(demand[i].type, demand[i].id);
		if(phy_sensor != NULL){
			if(equal_pi == 0){
				equal_pi = phy_sensor->pi;
				continue;
			}
			if(phy_sensor->pi != equal_pi){
				ko_flag++;
				break;
			}
			act++;
		}
	}
	if(act != 0 && ko_flag == 0)
		ret = 0;
	return ret;
}

static int CheckFeedIfDirect(feed_general_t* feed)
{
	sensor_data_demand_t* demand = feed->demand;
	uint8_t demand_length = feed->demand_length;
	if(feed->type != BASIC_ALGO_RAWDATA && demand_length == 1)
		return -1;

	if(feed->type == BASIC_ALGO_RAWDATA){
		int active_count = 0;
		for(int i = 0; i < demand_length; i++)
			if(demand[i].freq != 0)
				active_count++;

		if(active_count <= 1)
			return -1;
	}
	return 0;
}


void RefleshSensorCore(void)
{
	uint8_t active_flag = 0;
	uint8_t suspend_flag = 0;
	int motion_feed_on_count = 0;

	raw_data_dump_flag = 0;
	ResetPhySensorList();
	for(list_t* node = feed_list.head; node != NULL; node = node->next){
		feed_general_t* feed = (feed_general_t*)node;
		if((feed->stat_flag & ON) && feed->motion_sensor_flag)
			motion_feed_on_count++;	/* if the algo is subscribed and
                                     * it depends on motion sensor data,
                                     * then do NOT unsubcribe motion event */

		if((feed->stat_flag & ON) != 0 && ((feed->stat_flag & IDLE) == 0 || (feed->stat_flag & WAKE_UP_CLEAR_FIFO) != 0)){
			suspend_flag++;
			for(int i = 0; i < feed->demand_length; i++){
				sensor_data_demand_t* demand = &(feed->demand[i]);
				if(demand->freq != 0){
					sensor_handle_t* phy_sensor = GetPollSensStruct(demand->type, demand->id);
					if(phy_sensor != NULL){
						if(phy_sensor->fifo_length > 0){
							if(phy_sensor->freq == 0)
								phy_sensor->freq = MatchFreq(phy_sensor, demand->freq * 10);
							else{
								uint16_t temp_freq = MatchFreq(phy_sensor, demand->freq * 10);
								phy_sensor->freq = GetCommonMultiple(temp_freq, phy_sensor->freq);
							}
							if(phy_sensor->pi == 0)
								phy_sensor->pi = demand->rt;
							else
								phy_sensor->pi = demand->rt < phy_sensor->pi ?
									demand->rt : phy_sensor->pi;

							//check raw data feed if on and dump A or G
							if(feed->type == BASIC_ALGO_RAWDATA &&
								(demand->type == SENSOR_ACCELEROMETER || demand->type == SENSOR_GYROSCOPE))
								raw_data_dump_flag++;
						}else{
							if(phy_sensor->freq == 0)
								phy_sensor->freq = demand->freq * 10;
							else
								phy_sensor->freq = GetCommonMultiple(demand->freq * 10, phy_sensor->freq);
						}

						//turn on phy_sensor
						//hook phy_sensor to poll_active_link and poll_active_array
						if((phy_sensor->stat_flag & ON) == 0){
							phy_sensor->stat_flag |= ON;
							if(phy_sensor->type == SENSOR_OHRM)
								list_add_head(&phy_sensor_poll_active_list, &phy_sensor->links.poll.poll_active_link);
							else
								list_add(&phy_sensor_poll_active_list, &phy_sensor->links.poll.poll_active_link);

							int idx = GetHashKey(phy_sensor->type, phy_sensor->id);
							list_add(&phy_sensor_poll_active_array[idx], &phy_sensor->links.poll.poll_active_array_link);
						}

						if((feed->stat_flag & IDLE) == 0 && (phy_sensor->stat_flag & IDLE) == 0)
							phy_sensor->idle_ref++;

						active_flag++;
					}
				}
			}
		}
	}

	for(list_t* next = phy_sensor_list_poll.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next - offsetof(sensor_handle_t, links.poll.poll_link));
		if((phy_sensor->stat_flag & ON) == 0 && phy_sensor->dirty != 0){
			ClosePhySensor(phy_sensor);
			phy_sensor->dirty = 0;
		}
	}

	if(active_flag == 0 && motion_feed_on_count == 0){
		if(suspend_flag == 0 && global_suspend_flag == 0){
			//make bmi160 into suspend mode
			sensor_handle_t* phy_sensor = GetIntSensStruct(SENSOR_ANY_MOTION, DEFAULT_ID);
			if(phy_sensor != NULL && (phy_sensor->stat_flag & ON) != 0)
				CloseIntSensor(phy_sensor);

			phy_sensor = GetIntSensStruct(SENSOR_NO_MOTION, DEFAULT_ID);
			if(phy_sensor != NULL && (phy_sensor->stat_flag & ON) != 0)
				CloseIntSensor(phy_sensor);

			phy_sensor = GetIntSensStruct(SENSOR_TAPPING, DEFAULT_ID);
			if(phy_sensor != NULL && (phy_sensor->stat_flag & ON) != 0)
				CloseIntSensor(phy_sensor);

			global_suspend_flag = 1;
		}
		return;
	}

	double ct = get_uptime_32k() * (double)1000 / 32768;

	//set fifo phy sensor freq and no_fifo phy_sensor pi
	for(list_t* next = phy_sensor_poll_active_list.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next
			- offsetof(sensor_handle_t, links.poll.poll_active_link));
		if(phy_sensor->fifo_length > 0){
			uint16_t temp_freq = MatchFreq(phy_sensor, phy_sensor->freq);
			//set odr to fifo phy_sensor
			SetSensSamplingTime(phy_sensor, temp_freq);
		}else{
			phy_sensor->pi = (float)1000 * 10 / phy_sensor->freq;
		}
	}

	//work out the final pi of fifo phy_sensor
	for(list_t* next = phy_sensor_poll_active_list.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next
			- offsetof(sensor_handle_t, links.poll.poll_active_link));
		if(phy_sensor->fifo_length > 0){
			int final_pi = 0;
			float fifo_limit_pi = 0, algo_limit_pi = 0;
			if(phy_sensor->fifo_share_bitmap != 0 && phy_sensor->fifo_share_done_flag == 0){
				list_t* share_list_head = &phy_sensor->fifo_share_link;
				float mid_value = 0;
				do{
					sensor_handle_t* phy_sensor_node = (sensor_handle_t*)((void*)share_list_head
						- offsetof(sensor_handle_t, fifo_share_link));
					if((phy_sensor_node->stat_flag & ON) != 0){
						mid_value += (float)phy_sensor_node->freq
							* phy_sensor_node->sensor_data_raw_size;
						if(algo_limit_pi == 0 || algo_limit_pi > phy_sensor_node->pi)
							algo_limit_pi = phy_sensor_node->pi;
						phy_sensor_node->fifo_share_done_flag = 1;
					}
					share_list_head = share_list_head->next;
				}while(share_list_head != &phy_sensor->fifo_share_link);

				//got the share_fifo_phy_sensor fifo_limit_pi
				if(mid_value != 0)
					fifo_limit_pi = (float)phy_sensor->fifo_length * 1000 * 10 / mid_value - POLLING_TOLERANCE;

				final_pi = fifo_limit_pi > algo_limit_pi ? algo_limit_pi : fifo_limit_pi;
				do{
					sensor_handle_t* phy_sensor_node = (sensor_handle_t*)((void*)share_list_head
						- offsetof(sensor_handle_t, fifo_share_link));
					if((phy_sensor_node->stat_flag & ON) != 0)
						phy_sensor_node->pi = final_pi;
					share_list_head = share_list_head->next;
				}while(share_list_head != &phy_sensor->fifo_share_link);
			}else if(phy_sensor->fifo_share_bitmap == 0){
				algo_limit_pi = phy_sensor->pi;
				fifo_limit_pi = (float)phy_sensor->fifo_length * 1000 * 10
					/ phy_sensor->sensor_data_raw_size / phy_sensor->freq - POLLING_TOLERANCE;
				final_pi = fifo_limit_pi > algo_limit_pi ? algo_limit_pi : fifo_limit_pi;
				float si = (float)1000 * 10 / phy_sensor->freq;
				phy_sensor->pi = final_pi;
				if((phy_sensor->stat_flag & IDLE) != 0)
					phy_sensor->npp = ct + final_pi;
				if(final_pi > si && final_pi >= POLLING_TOLERANCE)
					phy_sensor->fifo_use_flag = 1;
			}
		}
	}

	//alloc sensor data match buffer to every demand of feed
	for(list_t* node = feed_list.head; node != NULL; node = node->next){
		feed_general_t* feed = (feed_general_t*)node;
		if((feed->stat_flag & ON) != 0 && ((feed->stat_flag & IDLE) == 0 || (feed->stat_flag & WAKE_UP_CLEAR_FIFO) != 0)){
			sensor_data_demand_t* demand = feed->demand;
			uint8_t demand_length = feed->demand_length;
			int ignore_flag = 0;
			if(0 > CheckFeedIfDirect(feed))
				continue;

			//work out common time consume of data collect
			float max_time_consume = 0;
			int cm_time_consume = 1;
			float d_min_buf_limit_pi = 0;
			float phy_sensors_max_pi = 0;

			for(int i = 0; i < demand_length; i++){
				if(demand[i].freq == 0)
					continue;
				sensor_handle_t* phy_sensor = GetActivePollSensStruct(demand[i].type, demand[i].id);
				if(phy_sensor != NULL){
					float buffer_limit_pi = (float)MATCH_BUFFER_LIMIT_SIZE * 1000
						/ phy_sensor->sensor_data_frame_size / demand[i].freq;
					if(d_min_buf_limit_pi == 0 || buffer_limit_pi < d_min_buf_limit_pi)
						d_min_buf_limit_pi = buffer_limit_pi;
				}
			}
			for(int i = 0; i < demand_length; i++){
				if(demand[i].freq == 0)
					continue;
				int d_si = ValueRound((float)1000 / demand[i].freq);
				if(cm_time_consume == 1 || cm_time_consume > d_si)
					cm_time_consume = d_si;
			}

			for(int i = 0; i < demand_length; i++){
				if(demand[i].freq == 0)
					continue;
				demand[i].flag &= ~IGNORE;
				int d_si = ValueRound((float)1000 / demand[i].freq);
				int temp = GetCommonMultiple(cm_time_consume, d_si);
				if(d_si >= d_min_buf_limit_pi || temp >= d_min_buf_limit_pi){
					demand[i].flag |= IGNORE;
					ignore_flag++;
					continue;
				}
				cm_time_consume = temp;
			}

			int min_check_ret = CheckMinDelayBuffer(feed);
			if(min_check_ret < 0){
				for(int i = 0; i < demand_length; i++){
					if(demand[i].freq == 0)
						continue;
					if((demand[i].flag & IGNORE) != 0)
						continue;
					sensor_handle_t* phy_sensor = GetActivePollSensStruct(demand[i].type, demand[i].id);
					if(phy_sensor != NULL){
						if(phy_sensor->pi > d_min_buf_limit_pi)
							phy_sensor->pi = d_min_buf_limit_pi;
						if(phy_sensor->pi > phy_sensors_max_pi)
							phy_sensors_max_pi = phy_sensor->pi;
					}
				}
			}

			//work out max_time_consume
			for(int i = 0; i < demand_length; i++){
				if(demand[i].freq == 0)
					continue;
				if((demand[i].flag & IGNORE) != 0)
					continue;
				sensor_handle_t* phy_sensor = GetActivePollSensStruct(demand[i].type, demand[i].id);
				if(phy_sensor != NULL){
					int times = 0;
					float mid_pi = 0;
					if(min_check_ret < 0){
						times = phy_sensors_max_pi / cm_time_consume <= 1 ? 1 : phy_sensors_max_pi / cm_time_consume;
					}else{
						times = 1;
					}

					mid_pi = cm_time_consume * times + POLLING_TOLERANCE;
					if(mid_pi > d_min_buf_limit_pi)
						mid_pi = d_min_buf_limit_pi;

					if(max_time_consume == 0 || max_time_consume < mid_pi)
						max_time_consume = mid_pi;
				}
			}

			//work out match_buffer_length to per demand
			for(int i = 0; i < demand_length; i++){
				if(demand[i].freq == 0)
					continue;
				sensor_handle_t* phy_sensor = GetActivePollSensStruct(demand[i].type, demand[i].id);
				if(phy_sensor != NULL){
					int d_si = ValueRound((float)1000 / demand[i].freq);
					int count = 0;
					if((demand[i].flag & IGNORE) == 0){
						count = (max_time_consume + cm_time_consume) / d_si;
					}else{
						count = d_si / phy_sensor->pi + 1;
					}
					demand[i].match_buffer_repo = count;
					ResetDemandDelayBuffer(&demand[i], count, phy_sensor);
				}
			}
		}
	}

	//work out pi_used_balloc for each physical sensor
	for(list_t* next = phy_sensor_poll_active_list.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next - offsetof(sensor_handle_t, links.poll.poll_active_link));
		if(phy_sensor->fifo_length > 0){
			phy_sensor->fifo_share_done_flag = 0;
			float buffer_limit_pi;
			if(raw_data_dump_flag == 0)
				buffer_limit_pi = (float)RAW_DATA_BUFFER_LIMIT_SIZE * 1000 * 10
					/ phy_sensor->sensor_data_frame_size / phy_sensor->freq;
			else
				buffer_limit_pi = (float)RAW_DATA_DUMP_BUFFER_SIZE * 1000 * 10
					/ phy_sensor->sensor_data_frame_size / phy_sensor->freq;

			float si =  (float)1000 * 10 / phy_sensor->freq;

			if(phy_sensor->pi + si >= buffer_limit_pi){
				phy_sensor->pi_used_balloc = buffer_limit_pi;
				phy_sensor->pi = buffer_limit_pi - si;
			}else if(phy_sensor->pi - si >= si){
				phy_sensor->pi_used_balloc = phy_sensor->pi + si;
			}else{
				phy_sensor->pi = si;
				phy_sensor->pi_used_balloc = si;
			}
		}
	}

	//make pi of fifo_share physical sensors equal to each other
	for(list_t* next = phy_sensor_poll_active_list.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next - offsetof(sensor_handle_t, links.poll.poll_active_link));
		if(phy_sensor->fifo_length > 0 && phy_sensor->fifo_share_bitmap != 0 && phy_sensor->fifo_share_done_flag == 0){
			list_t* share_list_head = &phy_sensor->fifo_share_link;
			float min_pi = 0;
			do{
				sensor_handle_t* phy_sensor_node = (sensor_handle_t*)((void*)share_list_head
					- offsetof(sensor_handle_t, fifo_share_link));
				if((phy_sensor_node->stat_flag & ON) != 0){
					if(min_pi == 0 || min_pi > phy_sensor_node->pi)
						min_pi = phy_sensor_node->pi;
					phy_sensor_node->fifo_share_done_flag = 1;
				}
				share_list_head = share_list_head->next;
			}while(share_list_head != &phy_sensor->fifo_share_link);

			do{
				sensor_handle_t* phy_sensor_node = (sensor_handle_t*)((void*)share_list_head
					- offsetof(sensor_handle_t, fifo_share_link));
				if((phy_sensor_node->stat_flag & ON) != 0){
					//set final_pi to phy_sensor on the fifo share list
					float si = (float)1000 * 10 / phy_sensor_node->freq;
					if(min_pi > si && min_pi >= POLLING_TOLERANCE){
						phy_sensor_node->pi = min_pi;
						if((phy_sensor_node->stat_flag & IDLE) != 0)
							phy_sensor->npp = ct + min_pi;
						phy_sensor_node->fifo_use_flag = 1;
					}else{
						phy_sensor_node->pi =  si;
						if((phy_sensor_node->stat_flag & IDLE) != 0)
							phy_sensor->npp = ct + si;
					}
				}
				share_list_head = share_list_head->next;
			}while(share_list_head != &phy_sensor->fifo_share_link);
		}
	}

	//alloc raw sensor data buffer to phy sensors
	for(list_t* next = phy_sensor_poll_active_list.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next - offsetof(sensor_handle_t, links.poll.poll_active_link));
		int len = 0, buffer_length_changed = 0;
		if(phy_sensor->fifo_length > 0 && phy_sensor->fifo_use_flag != 0)
			len = phy_sensor->pi_used_balloc / ((float)1000 * 10 / phy_sensor->freq);
		else
			len = 1;

		if(phy_sensor->sensor_data_frame_size * len != phy_sensor->buffer_length){
			if(phy_sensor->buffer != NULL){
				if(phy_sensor->buffer_length / phy_sensor->sensor_data_frame_size == 1){
					void* temp_ptr = phy_sensor->buffer - offsetof(struct sensor_data, data);
					bfree(temp_ptr);
					if(len > 1)
						phy_sensor_enable(phy_sensor->ptr, 0);
				}else{
					FreeInDss(phy_sensor->buffer);
					if(len == 1)
						phy_sensor_enable_hwfifo(phy_sensor->ptr, 0, 0);
				}
				phy_sensor->buffer = NULL;
				phy_sensor->buffer_length = 0;
			}

			uint32_t alloc_length;
			if(len == 1){
				int length = sizeof(struct sensor_data) + phy_sensor->sensor_data_frame_size;
				struct sensor_data* temp_ptr = NULL;
				temp_ptr = (struct sensor_data*)balloc(length, NULL);
				if(temp_ptr == NULL)
					continue;

				temp_ptr->data_length = phy_sensor->sensor_data_frame_size;
				phy_sensor->buffer = (void*)temp_ptr->data;
				alloc_length = len * phy_sensor->sensor_data_frame_size;
			}else{
				if(raw_data_dump_flag == 0 && phy_sensor->sensor_data_frame_size * len > RAW_DATA_BUFFER_LIMIT_SIZE / 2)
					alloc_length = RAW_DATA_BUFFER_LIMIT_SIZE;
				else if(raw_data_dump_flag == 0 && phy_sensor->sensor_data_frame_size * len > RAW_DATA_BUFFER_LIMIT_SIZE / 4)
					alloc_length = RAW_DATA_BUFFER_LIMIT_SIZE / 2;
				else
					alloc_length = phy_sensor->sensor_data_frame_size * len;

				phy_sensor->buffer = AllocFromDss(alloc_length);
				if(phy_sensor->buffer == NULL)
					continue;
			}
			phy_sensor->buffer_length = alloc_length;
			buffer_length_changed++;
		}

		pr_debug(LOG_MODULE_OPEN_CORE, "phytype=%d, buflen=%d, pi=%d, freq=%d", phy_sensor->type, phy_sensor->buffer_length, (uint32_t)phy_sensor->pi, phy_sensor->freq);

		phy_sensor->need_poll = 1;
		if(phy_sensor->fifo_length > 0 && phy_sensor->fifo_use_flag != 0){
#ifdef SUPPORT_INTERRUPT_MODE
			//if support fifo int, register cb, need_poll = 0
			if((phy_sensor->attri_mask & PHY_SENSOR_REPORT_MODE_INT_FIFO_MASK) != 0){
				phy_sensor_watermark_property_t temp_prop = {
					.count = phy_sensor->pi / ((float)1000 * 10 / phy_sensor->freq),
					.callback = raw_data_fifo_int_cb,
					.priv_data = (void*)phy_sensor,
					};
				phy_sensor_set_property(phy_sensor->ptr, SENSOR_PROP_FIFO_WATERMARK, &temp_prop);
				phy_sensor->need_poll = 0;
			}
#endif
			if(phy_sensor->dirty == 0 || (phy_sensor->dirty != 0 && buffer_length_changed != 0))
				phy_sensor_enable_hwfifo_with_buffer(phy_sensor->ptr, 1,
					(uint8_t *)phy_sensor->buffer, phy_sensor->buffer_length);
		}else{
			if(phy_sensor->dirty == 0)
				phy_sensor_enable(phy_sensor->ptr, 1);
#ifdef SUPPORT_INTERRUPT_MODE
			//if support reg int, register cb, need_poll = 0
			if((phy_sensor->attri_mask & PHY_SENSOR_REPORT_MODE_INT_REG_MASK) != 0){
				phy_sensor_data_register_callback(phy_sensor->ptr, raw_data_reg_int_cb, NULL, phy_sensor->freq);
				phy_sensor->need_poll = 0;
			}
#endif
		}
		phy_sensor->dirty = 0;
	}
}

#ifdef SUPPORT_INTERRUPT_MODE
static void raw_data_fifo_int_cb(phy_sensor_event_t* event, void* priv_data)
{
	int param_length = sizeof(void*);
	int msg_length = sizeof(struct ia_cmd) + param_length;
	struct ia_cmd* cmd = (struct ia_cmd*)balloc(msg_length, NULL);

	memcpy(cmd->param, &priv_data, param_length);
	cmd->length = param_length;
	cmd->cmd_id = CMD_RAWDATA_FIFO_INT_SC;
	ipc_2core_send(cmd);
	return;
}

static int raw_data_reg_int_cb(struct sensor_data* sensor_data, void* priv_data)
{
	int msg_length = sizeof(struct ia_cmd) + sizeof(struct sensor_data) + sensor_data->data_length;
	struct ia_cmd* cmd = (struct ia_cmd*)balloc(msg_length, NULL);

	memcpy(cmd->param, sensor_data, sizeof(struct sensor_data) + sensor_data->data_length);
	cmd->cmd_id = CMD_RAWDATA_REG_INT_SC;
	ipc_2core_send(cmd);
	return 0;
}
#endif

int motion_detect_callback(struct sensor_data* sensor_data, void* priv_data)
{
	uint8_t* event = sensor_data->data;
	uint8_t id;
	uint8_t act = 0;
	if(*event == PHY_SENSOR_EVENT_ANY_MOTION){
		id = CMD_RESUME_SC;
		act++;
	}else if(*event == PHY_SENSOR_EVENT_NO_MOTION){
		id = CMD_SUSPEND_SC;
		act++;
	}else if(*event == PHY_SENSOR_EVENT_DOUBLE_TAP){
		struct tapping_result value;
		value.tapping_cnt = (int16_t)2;
		OpencoreCommitSensData(SENSOR_ABS_TAPPING, DEFAULT_ID, (void*)&value, sizeof(struct tapping_result));
		id = CMD_RESUME_SC;
		act++;
	}

	if(act != 0){
		struct ia_cmd* cmd = (struct ia_cmd*)balloc(sizeof(struct ia_cmd), NULL);
		if(cmd == NULL)
			return -1;

		memset(cmd, 0, sizeof(struct ia_cmd));
		cmd->cmd_id = id;
		ipc_2core_send(cmd);
	}
	return 0;
}

static void PMInit(void)
{
	global_suspend_flag = 0;
	pm_wakelock_init(&opencore_main_wl);
	pm_wakelock_init(&opencore_cali_wl);
}

static void PhySensorsInit(void)
{
	int scan_ret = 0;
	int count = 0;
	uint64_t bitmap = ~((uint64_t)0);
	sensor_id_t sens_list[MAX_PHY_SENSOR_NUM];
	scan_ret = get_sensor_list(bitmap, sens_list, MAX_PHY_SENSOR_NUM);
	for(int i = 0; i < scan_ret; i++){
		sensor_t phy_sensor_handle = phy_sensor_open(sens_list[i].sensor_type, sens_list[i].dev_id);
		if(phy_sensor_handle != NULL){
			uint8_t mask = 0;
			sensor_handle_t* phy_sensor = balloc(sizeof(sensor_handle_t), NULL);
			if(phy_sensor == NULL)
				continue;

			memset(phy_sensor, 0, sizeof(sensor_handle_t));
			phy_sensor->type = sens_list[i].sensor_type;
			phy_sensor->id = sens_list[i].dev_id;
			phy_sensor->ptr = phy_sensor_handle;
			phy_sensor_get_report_mode_mask(phy_sensor_handle, &mask);
			phy_sensor->attri_mask = mask;

			if((mask & (PHY_SENSOR_REPORT_MODE_POLL_REG_MASK | PHY_SENSOR_REPORT_MODE_POLL_FIFO_MASK) ) != 0){
				int ret_prop;
				phy_sensor_get_hw_raw_data_len(phy_sensor_handle, &phy_sensor->sensor_data_raw_size);
				phy_sensor_get_raw_data_len(phy_sensor_handle, &phy_sensor->sensor_data_frame_size);
				phy_sensor->fifo_length = phy_sensor_data_get_hwfifo_length(phy_sensor->ptr);
				if(phy_sensor->fifo_length > 0){
					phy_sensor->mutex = mutex_create();
					ShareListInit(&phy_sensor->fifo_share_link);
					//get fifo_share_infomation
					phy_sensor_fifo_share_property_t fifo_share;
					phy_sensor_get_property(phy_sensor->ptr, SENSOR_PROP_FIFO_SHARE_BITMAP, &fifo_share);
					phy_sensor->fifo_share_bitmap = fifo_share.bitmap;

					//hook each other with fifo share bitmap
					for(int i = 0; i < 32 && phy_sensor->fifo_share_bitmap != 0; i++){
						if((phy_sensor->fifo_share_bitmap & (1 << i)) != 0){
							for(list_t* next = phy_sensor_list_poll.head; next != NULL; next = next->next){
								sensor_handle_t* share_phy_sensor =
									(sensor_handle_t*)((void*)next - offsetof(sensor_handle_t, links.poll.poll_link));
								if(share_phy_sensor->id == i){
									ShareListAdd(&share_phy_sensor->fifo_share_link, &phy_sensor->fifo_share_link);
									goto out;
								}
							}
						}
					}
				}

				//get phy_sensor range_ max and range_min
				phy_sensor_range_property_t sensing_range;
out:
				ret_prop = phy_sensor_get_property(phy_sensor->ptr, SENSOR_PROP_SENSING_RANGE, &sensing_range);
				if(ret_prop == 0){
					phy_sensor->range_max = sensing_range.high;
					phy_sensor->range_min = sensing_range.low;
				}

				phy_sensor->clb_data_buffer = balloc(phy_sensor->sensor_data_frame_size, NULL);
				if(phy_sensor->clb_data_buffer == NULL)
					continue;

				memset(phy_sensor->clb_data_buffer, 0, phy_sensor->sensor_data_frame_size);

				phy_sensor->feed_data_buffer = balloc(phy_sensor->sensor_data_frame_size, NULL);
				if(phy_sensor->feed_data_buffer == NULL)
					continue;

				memset(phy_sensor->feed_data_buffer, 0, phy_sensor->sensor_data_frame_size);
				list_add(&phy_sensor_list_poll, &phy_sensor->links.poll.poll_link);
				count++;
			}else{
				if(sens_list[i].sensor_type == SENSOR_ANY_MOTION
					|| sens_list[i].sensor_type == SENSOR_NO_MOTION){
					OpenIntSensor(phy_sensor);
				}

				list_add(&phy_sensor_list_int, &phy_sensor->links.int_link);
			}
		}
	}

	//balloc exposed_sensor_t
	sensor_data_demand_t* demand_ptr = balloc(sizeof(sensor_data_demand_t) * count, NULL);
	if(demand_ptr == NULL)
		return;

	atlsp_algoC.demand = demand_ptr;
	atlsp_algoC.demand_length = count;

	for(list_t* next = phy_sensor_list_poll.head; next != NULL; next = next->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next - offsetof(sensor_handle_t, links.poll.poll_link));
		demand_ptr[count - 1].type = phy_sensor->type;
		demand_ptr[count - 1].id = phy_sensor->id;
		count--;

		exposed_sensor_t* exposed_sensor = balloc(sizeof(exposed_sensor_t), NULL);
		if(exposed_sensor == NULL)
			continue;

		exposed_sensor->depend_flag = 1 << BASIC_ALGO_RAWDATA;
		exposed_sensor->type = phy_sensor->type;
		exposed_sensor->id = phy_sensor->id;
		exposed_sensor->stat_flag = DIRECT_RAW;

		list_add(&exposed_sensor_list, (list_t*)exposed_sensor);
	}
}

static void FeedInit(void)
{
	long* feed_p = (long*)_s_feedinit;
	while(feed_p < (long*)_e_feedinit){
		feed_general_t* feed = (feed_general_t*)(*feed_p);
		if(feed == NULL)
			continue;

		if(feed->type != (uint8_t)BASIC_ALGO_RAWDATA){
			sensor_data_demand_t* demand = feed->demand;
			for(int i = 0; i < feed->demand_length; i++){
				int type_match_flag = 0, id_match_flag = 0;
				sensor_handle_t* phy_sensor_match_ptr = NULL;
				int range_match_area = 0;

				if( demand[i].type == SENSOR_ACCELEROMETER ||
					demand[i].type == SENSOR_GYROSCOPE ||
					demand[i].type == SENSOR_MAGNETOMETER)
					feed->motion_sensor_flag = 1;	/* Mark the algo depends on motion sensor data*/

				for(list_t* next = phy_sensor_list_poll.head; next != NULL; next = next->next){
					//	void* temp_ptr = phy_sensor[core_type]->buffer - offsetof(struct sensor_data, data);
					sensor_handle_t* phy_sensor =
						(sensor_handle_t*)((void*)next - offsetof(sensor_handle_t, links.poll.poll_link));

					if(phy_sensor->type == demand[i].type){
						type_match_flag++;
						if(demand[i].id != DEFAULT_ID && phy_sensor->id == demand[i].id)
							id_match_flag++;
						else if(demand[i].id == DEFAULT_ID){
							//get match range area
							int16_t area_from = 0, area_to = 0, area = 0;

							if(demand[i].range_min == DEFAULT_VALUE || demand[i].range_min < phy_sensor->range_min)
								area_from = phy_sensor->range_min;
							else
								area_from = demand[i].range_min;

							if(demand[i].range_max == DEFAULT_VALUE || demand[i].range_max > phy_sensor->range_max)
								area_to = phy_sensor->range_max;
							else
								area_to = demand[i].range_max;

							area = area_to - area_from;

							//if large than prev , then replace prev ptr
							if((range_match_area == 0 || area > range_match_area) && area >= 0)
								phy_sensor_match_ptr = phy_sensor;
						}
					}
				}

				if(type_match_flag == 0)
					goto skip;

				if(demand[i].id != DEFAULT_ID && id_match_flag == 0)
					goto skip;

				if(demand[i].id == DEFAULT_ID && phy_sensor_match_ptr == NULL)
					goto skip;

				if(phy_sensor_match_ptr != NULL)
					demand[i].id = phy_sensor_match_ptr->id;

				if(demand[i].freq <= 0 || demand[i].rt <= 0)
					goto skip;

				float si = (float)1000 / demand[i].freq;
				if(demand[i].rt < si)
					goto skip;
			}
		}

		if(feed->type != (uint8_t)BASIC_ALGO_OHRM)
			list_add(&feed_list, (list_t*)feed);
		else
			list_add_head(&feed_list, (list_t*)feed);
skip:
		feed_p++;
	}
}

static void ExposedSensorInit(void)
{
	long* sensor_p = (long*)_s_exposedinit;
	while(sensor_p < (long*)_e_exposedinit){
		int valid = 0;
		exposed_sensor_t* exposed_sensor = (exposed_sensor_t*)(*sensor_p);
		if(exposed_sensor == NULL)
			continue;

		for(list_t* node = feed_list.head; node != NULL; node = node->next){
			feed_general_t* feed = (feed_general_t*)node;
			if(exposed_sensor->depend_flag == (1 << feed->type)){
				valid++;
				break;
			}
		}

		if(valid != 0)
			list_add(&exposed_sensor_list, (list_t*)exposed_sensor);
		sensor_p++;
	}
}

void SensorCoreInit(void)
{
	PhySensorsInit();
	FeedInit();
	ExposedSensorInit();
	PMInit();
	RefleshSensorCore();
}
/* *INDENT-ON* */
