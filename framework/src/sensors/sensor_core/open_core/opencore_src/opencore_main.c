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
static int poll_timeout = OS_WAIT_FOREVER;
static char stack[STACKSIZE] __aligned(4);
static uint8_t no_motion_flag;
static uint8_t calibration_flag;
int read_out_fifo_flag = 0;
static int loop;
static int reg_mark;
static int fifo_mark;
static uint32_t read_ohrm_consume;
static list_head_t cmd_head;

#define THRESHOLD_CMD_COUNT 10
static int SensorCoreProcess(int* poll_timeout)
{
	loop = reg_mark = fifo_mark = read_ohrm_consume = 0;
	static double last_ct = 0;
	static int last_poll_timeout = 0;
	int act_npp, act_algo = 0;
	while(1){
		double min_npp = 0;
		double ct = get_uptime_32k() * (double)1000 / 32768;
		if(ct < last_ct)
			RefleshSensorCore();

		last_ct = ct;
		act_npp = 0;
		for(list_t* next = phy_sensor_poll_active_list.head; next != NULL; next = next->next){
			sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)next
				- offsetof(sensor_handle_t, links.poll.poll_active_link));
			if((phy_sensor->stat_flag & IDLE) != 0)
				continue;

			if(phy_sensor->fifo_length > 0 && phy_sensor->fifo_use_flag != 0 && read_out_fifo_flag != 0)
				continue;

			if(phy_sensor->need_poll != 0){
				if(min_npp == 0 || min_npp > phy_sensor->npp)
					min_npp = phy_sensor->npp;
				act_npp++;
			}
		}

		if(act_npp == 0){
			last_poll_timeout = FOREVER_VALUE;
			*poll_timeout = FOREVER_VALUE;
			return act_algo;
		}

		if(ct < min_npp && min_npp - ct > 1){
			last_poll_timeout = min_npp - ct;
			*poll_timeout = min_npp - ct;
			return act_algo;
		}

		loop++;
		act_algo = 0;
		for(list_t* node = phy_sensor_poll_active_list.head; node != NULL; node = node->next){
			sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)node - offsetof(sensor_handle_t, links.poll.poll_active_link));
			if((phy_sensor->stat_flag & IDLE) != 0)
				continue;
			if(phy_sensor->need_poll != 0 && ct >= phy_sensor->npp && phy_sensor->buffer != NULL){
				int ret = 0;
				if(phy_sensor->fifo_length > 0 && phy_sensor->fifo_use_flag != 0 && read_out_fifo_flag == 0){
					//get node and buffer
					TriggerAlgoEngine(READ_FIFO, (void*)phy_sensor);
					phy_sensor->npp = ct + phy_sensor->pi;
					act_algo++;
					fifo_mark++;
				}else if(phy_sensor->fifo_use_flag == 0){
					void* temp_ptr = phy_sensor->buffer - offsetof(struct sensor_data, data);
					int ct_local = get_uptime_ms();
					ret = phy_sensor_data_read(phy_sensor->ptr, (struct sensor_data*)temp_ptr);
					read_ohrm_consume = get_uptime_ms() - ct_local;
					if(ret != 0){
						raw_data_node_t* node = (raw_data_node_t*)AllocFromDss(sizeof(raw_data_node_t));
						if(node == NULL)
							pr_error(LOG_MODULE_OPEN_CORE, "fail to alloc raw data node reg");
						void* buffer = AllocFromDss(phy_sensor->buffer_length);
						if(buffer == NULL)
							pr_error(LOG_MODULE_OPEN_CORE, "fail to alloc raw data buffer reg");
						if(node != NULL && buffer != NULL){
							memcpy(buffer, phy_sensor->buffer, ret);
							node->buffer = buffer;
							node->raw_data_count = ret / phy_sensor->sensor_data_frame_size;
							uint8_t head_for_raw = phy_sensor->head_for_algo == 0 ? 1 : 0;
							list_add(&phy_sensor->raw_data_head[head_for_raw], &node->raw_data_node);
							act_algo++;
						}else{
							if(node != NULL)
								FreeInDss((void*)node);
							if(buffer != NULL)
								FreeInDss(buffer);
						}
					}
					phy_sensor->npp = ct + phy_sensor->pi;
					reg_mark++;
				}
			}
		}
	}
}

static struct ia_cmd* CoreSensorControl(struct sensor_id* sensor_id, core_sensor_ctl_t ctl)
{
	int act1, act2;
	int length = 0;
	int get_property_resp_param_length = 0;
	static uint8_t type_save;
	static int16_t id_save;
	static uint16_t exposed_stat_flag_save;
	static uint16_t feed_stat_flag_save;
	static sensor_data_demand_t* demand_array_save;
	struct ia_cmd* resp = NULL;
	struct return_value* rv;
	switch(ctl){
		case ALGO_CALIBRATION_STOP:
		case ALGO_CALIBRATION_SET:
		case ALGO_CALIBRATION_START:
			length = sizeof(struct ia_cmd) + sizeof(struct resp_calibration);
			break;
		case ALGO_CALIBRATION_PROCESS:
			{
				sensor_handle_t* phy_sensor = GetPollSensStruct(sensor_id->sensor_type, sensor_id->dev_id);
				if(phy_sensor != NULL)
					length = sizeof(struct ia_cmd) + sizeof(struct resp_calibration)
						+ phy_sensor->sensor_data_frame_size;
				else
					return NULL;
			}
			break;
		case ALGO_GET_PROPERTY:
			{
				uint8_t	property_type = ((struct get_property*)sensor_id)->property;
				/* according to property_type, define the length of  property_param of struct resp_get_property
					here, for example, assume the length is uint32_t */
				switch(property_type){
					default:
						length = sizeof(struct ia_cmd) + sizeof(struct resp_calibration) + sizeof(uint32_t);
						get_property_resp_param_length = sizeof(uint32_t);
						break;
				}
			}
			break;
		default:
			length = sizeof(struct ia_cmd) + sizeof(struct return_value);
			break;
	}

	resp = (struct ia_cmd*)balloc(length, NULL);
	if(resp == NULL)
		return NULL;

	memset(resp, 0, length);
	resp->length = length;
	rv = (struct return_value*)resp->param;
	memcpy(&(rv->sensor), sensor_id, sizeof(struct sensor_id));
	rv->ret = RESP_SUCCESS;

	if(ctl == ALGO_GET_PROPERTY)
		((struct resp_get_property*)rv)->length = get_property_resp_param_length;

	for(list_t* next = exposed_sensor_list.head; next != NULL; next = next->next){
		exposed_sensor_t* exposed_sensor = (exposed_sensor_t*)next;
		if(exposed_sensor->type == sensor_id->sensor_type && exposed_sensor->id == sensor_id->dev_id){
			switch(ctl){
				case ALGO_SUBSCRIBE:
					{
						int	freq = ((struct subscription*)sensor_id)->sampling_interval;
						int	rt = ((struct subscription*)sensor_id)->reporting_interval;
						int rf_cnt_flag = 0;
						sensor_handle_t* phy_sensor = GetPollSensStruct(sensor_id->sensor_type, sensor_id->dev_id);
						if(phy_sensor != NULL){
							phy_sensor->npp = 0;
							if((phy_sensor->stat_flag&IDLE) != 0 && (exposed_sensor->stat_flag & DIRECT_RAW) != 0)
								phy_sensor->stat_flag &= ~IDLE;
						}
						if((exposed_sensor->stat_flag & DIRECT_RAW) != 0){
							if(calibration_flag == 1 || rt <= 0 || freq <= 0 || (1000 / freq) > rt){
								rv->ret = RESP_DEVICE_EXCEPTION;
								goto out;
							}
						}

						if((exposed_sensor->stat_flag & ON) != 0)
							rf_cnt_flag++;
						else
							exposed_sensor->stat_flag |= ON | SUBSCRIBED;

						act1 = act2 = 0;
						for(list_t* next = feed_list.head; next != NULL; next = next->next){
							feed_general_t* feed = (feed_general_t*)next;
							if((1 << feed->type & exposed_sensor->depend_flag) != 0){
								if((exposed_sensor->stat_flag & DIRECT_RAW) != 0){
									for(int i = 0; i < feed->demand_length; i++)
										if(feed->demand[i].type == exposed_sensor->type &&
												feed->demand[i].id == exposed_sensor->id){
											feed->demand[i].freq = freq;
											feed->demand[i].rt = rt;
											feed->demand[i].raw_data_offset = 0;
											if(feed->ctl_api.reset != NULL)
												feed->ctl_api.reset(feed);
											act1++;
									}
								}
								if((feed->stat_flag & ON) == 0){
									feed->stat_flag |= ON;
									if(global_suspend_flag == 1){
										sensor_handle_t* phy_sensor = GetIntSensStruct(SENSOR_ANY_MOTION, DEFAULT_ID);
										if(phy_sensor != NULL)
											OpenIntSensor(phy_sensor);

										phy_sensor = GetIntSensStruct(SENSOR_NO_MOTION, DEFAULT_ID);
										if(phy_sensor != NULL)
											OpenIntSensor(phy_sensor);

										global_suspend_flag = 0;
									}

									if(feed->ctl_api.init != NULL)
										feed->ctl_api.init(feed);

									for(int i = 0; i < feed->demand_length; i++)
										feed->demand[i].raw_data_offset = 0;
									act1++;
								}
								if(rf_cnt_flag == 0)
									feed->rf_cnt++;
								act2++;
							}
						}

						if(act2 == 0)
							rv->ret = RESP_DEVICE_EXCEPTION;

						if(act1 != 0)
							goto refresh;

						goto out;
					}
				case ALGO_UNSUBSCRIBE:
					{
						int rf_cnt_flag = 0;
						int clear_idle_flag = 0;
						if((exposed_sensor->stat_flag & ON) != 0)
							exposed_sensor->stat_flag &= ~(SUBSCRIBED | ON);
						else
							rf_cnt_flag++;
						act1 = act2 = 0;
						for(list_t* next = feed_list.head; next != NULL; next = next->next){
							feed_general_t* feed = (feed_general_t*)next;
							if((1 << feed->type & exposed_sensor->depend_flag) != 0){
								if((exposed_sensor->stat_flag & DIRECT_RAW) != 0){
									for(int i = 0; i < feed->demand_length; i++)
										if(feed->demand[i].type == exposed_sensor->type &&
												feed->demand[i].id == exposed_sensor->id){
											feed->demand[i].freq = 0;
											feed->demand[i].rt = 0;

											if(feed->ctl_api.reset != NULL)
												feed->ctl_api.reset(feed);
											act1++;
										}
								} else {
									if((feed->stat_flag & IDLE) == 0)
										clear_idle_flag++;
								}
								if(rf_cnt_flag == 0)
									feed->rf_cnt--;
								if(feed->rf_cnt == 0){
									feed->stat_flag &= ~ON;
									act1++;
								}
								act2++;
							}
						}
						if(!clear_idle_flag && no_motion_flag)
						{
							sensor_handle_t* phy_sensor = GetPollSensStruct(sensor_id->sensor_type, sensor_id->dev_id);
							if(phy_sensor != NULL)
								phy_sensor->stat_flag |= IDLE;
						}
						if(act2 == 0)
							rv->ret = RESP_DEVICE_EXCEPTION;

						if(act1 != 0)
							goto refresh;

						goto out;
					}
				case ALGO_GET_PROPERTY:
					{
						act1 = 0;
						for(list_t* next = feed_list.head; next != NULL; next = next->next){
							feed_general_t* feed = (feed_general_t*)next;
							if((1 << feed->type & exposed_sensor->depend_flag) != 0 && (feed->stat_flag & ON) != 0){
								if(feed->ctl_api.get_property != NULL){
									uint8_t	property_type = ((struct get_property*)sensor_id)->property;
									struct resp_get_property* ptr_resp = (struct resp_get_property*)rv;
									feed->ctl_api.get_property(feed, sensor_id->sensor_type, sensor_id->dev_id,
										property_type, (void*)ptr_resp->property_params);
								}
								act1++;
							}
						}

						if(act1 == 0)
							rv->ret = RESP_DEVICE_EXCEPTION;

						goto out;
					}
				case ALGO_SET_PROPERTY:
					{
						int ret;
						act1 = 0;
						for(list_t* next = feed_list.head; next != NULL; next = next->next){
							feed_general_t* feed = (feed_general_t*)next;
							if((1 << feed->type & exposed_sensor->depend_flag) != 0){
								if(feed->ctl_api.set_property != NULL){
									struct set_property* ptr = (struct set_property*)sensor_id;
									ret = feed->ctl_api.set_property(feed, sensor_id->sensor_type, sensor_id->dev_id,
										ptr->length, (void*)ptr->property_params);
									if(ret < 0)
										rv->ret = RESP_INVALID_PARAM;
								}
								act1++;
							}
						}

						if(act1 == 0)
							rv->ret = RESP_DEVICE_EXCEPTION;

						goto out;
					}
					break;
				case ALGO_CALIBRATION_SET:
					{
						act1 = 0;
						if(calibration_flag == 1)
							break;
						switch(exposed_sensor->type){
							case SENSOR_ACCELEROMETER:
								{
									sensor_handle_t* phy_sensor =
										GetPollSensStruct(exposed_sensor->type, exposed_sensor->id);
									if(phy_sensor != NULL){
										short* cali_value = (short*)(((struct calibration*)sensor_id)->calib_params);
										short* ptr = (short*)phy_sensor->clb_data_buffer;
										if(ptr == NULL)
											break;
										for(int i = 0; i < 3; i++)
											ptr[i] = cali_value[i];
										act1++;
									}
								}
								break;
							case SENSOR_GYROSCOPE:
								{
									sensor_handle_t* phy_sensor =
										GetPollSensStruct(exposed_sensor->type, exposed_sensor->id);
									if(phy_sensor != NULL){
										int* cali_value = (int*)(((struct calibration*)sensor_id)->calib_params);
										int* ptr = (int*)phy_sensor->clb_data_buffer;
										if(ptr == NULL)
											break;
										for(int i = 0; i < 3; i++)
											ptr[i] = cali_value[i];
										act1++;
									}
								}
								break;
							default:
								break;
						}
						if(act1 != 0)
							goto out;
					}
					break;
				case ALGO_CALIBRATION_START:
					{
						act1 = 0;
						if(calibration_flag == 1)
							break;

						if(exposed_sensor->type != SENSOR_ACCELEROMETER && exposed_sensor->type != SENSOR_GYROSCOPE)
							break;

						for(list_t* next = feed_list.head; next != NULL; next = next->next){
							feed_general_t* feed = (feed_general_t*)next;
							if((1 << feed->type & exposed_sensor->depend_flag) != 0){
								demand_array_save = AllocFromDss(sizeof(sensor_data_demand_t) * feed->demand_length);
								if(demand_array_save == NULL)
									break;

								memcpy(demand_array_save, feed->demand, feed->demand_length * sizeof(sensor_data_demand_t));
								for(int i = 0; i < feed->demand_length; i++){
									feed->demand[i].freq = feed->demand[i].rt = 0;
									if(feed->demand[i].type ==	exposed_sensor->type && feed->demand[i].id == exposed_sensor->id){
										feed->demand[i].freq = 1;
										feed->demand[i].rt = 1000;
										if(feed->ctl_api.reset != NULL)
											feed->ctl_api.reset(feed);
									}
								}
								type_save = exposed_sensor->type;
								id_save = exposed_sensor->id;
								calibration_flag = 1;
								pm_wakelock_acquire(&opencore_cali_wl);
								exposed_stat_flag_save = exposed_sensor->stat_flag;
								exposed_sensor->stat_flag |= ON;
								feed_stat_flag_save = feed->stat_flag;
								feed->stat_flag &= ~IDLE;
								feed->stat_flag |= ON;
								feed->idle_hold_flag = 1;

								act1++;
							}
						}
						if(act1 != 0)
							goto refresh;
					}
					break;
				case ALGO_CALIBRATION_PROCESS:
					{
						if(calibration_flag == 0 || no_motion_flag != 1)
							break;
						if(exposed_sensor->type != type_save || exposed_sensor->id != id_save)
							break;
						if(raw_data_calibration_flag == 1)
							break;
						raw_data_calibration_flag = 1;
						goto out;
					}
				case ALGO_CALIBRATION_STOP:
					{
						act1 = 0;
						if(calibration_flag == 0)
							break;
						if(exposed_sensor->type != type_save || exposed_sensor->id != id_save)
							break;
						if(raw_data_calibration_flag == 1)
							break;
						for(list_t* next = feed_list.head; next != NULL; next = next->next){
							feed_general_t* feed = (feed_general_t*)next;
							if((1 << feed->type & exposed_sensor->depend_flag) != 0){
								feed->stat_flag = feed_stat_flag_save;
								if(no_motion_flag == 1){
									struct ia_cmd* cmd = (struct ia_cmd*)balloc(sizeof(struct ia_cmd), NULL);
									if(cmd != NULL){
										memset(cmd, 0, sizeof(struct ia_cmd));
										cmd->cmd_id = CMD_SUSPEND_SC;
										ipc_2core_send(cmd);
									}
								}else
									feed->stat_flag &= ~IDLE;

								feed->idle_hold_flag = 0;
								if(demand_array_save != NULL){
									memcpy(feed->demand, demand_array_save,
										feed->demand_length * sizeof(sensor_data_demand_t));
									FreeInDss(demand_array_save);
									demand_array_save = NULL;
								}

								if(feed->ctl_api.reset != NULL)
									feed->ctl_api.reset(feed);
								act1++;
							}
						}
						exposed_sensor->stat_flag = exposed_stat_flag_save;
						calibration_flag = 0;
						if(act1 != 0)
							goto refresh;
					}
					break;
				default:
					break;
			}
		}
	}
	rv->ret = RESP_NO_DEVICE;
	return resp;
refresh:
	RefleshSensorCore();
out:
	return resp;
}

static struct ia_cmd *GetCoreSensorList(uint32_t bitmap)
{
	int count = 0, index = 0;
	for(list_t* next = exposed_sensor_list.head; next != NULL; next = next->next){
		exposed_sensor_t* exposed_sensor = (exposed_sensor_t*)next;
		if((bitmap & (1 << exposed_sensor->type)) != 0)
			count++;
	}

	int len = sizeof(struct ia_cmd) + sizeof(struct sensor_list) + sizeof(struct sensor_id) * count;
	struct ia_cmd *resp = (struct ia_cmd*)balloc(len, NULL);
	if(resp == NULL)
		return NULL;

	memset(resp, 0, len);
	resp->length = len;
	struct sensor_list* list = (struct sensor_list*)resp->param;
	list->count = count;

	for(list_t* next = exposed_sensor_list.head; next != NULL; next = next->next){
		exposed_sensor_t* exposed_sensor = (exposed_sensor_t*)next;
		if((bitmap & (1 << exposed_sensor->type)) != 0){
			list->sensor_list[index].sensor_type = exposed_sensor->type;
			list->sensor_list[index].dev_id = exposed_sensor->id;
			index++;
		}
	}
	return resp;
}

static void PrepareSWtappingDetect(void)
{
	//check if tapping algo is on
	//if on, check if accel is fifo enable
	//if fifo disable, balloc buffer to accel sensor and enable accel fifo
	for(list_t* next = feed_list.head; next != NULL; next = next->next){
		feed_general_t* feed = (feed_general_t*)next;
		if(feed->type == BASIC_ALGO_TAPPING && (feed->stat_flag & ON) != 0){
			for(int i = 0; i < feed->demand_length; i++){
				sensor_data_demand_t* demand = &(feed->demand[i]);
				if(demand->type == SENSOR_ACCELEROMETER){
					sensor_handle_t* phy_sensor = GetPollSensStruct(demand->type, demand->id);
					if(phy_sensor != NULL && phy_sensor->fifo_length > 0 && phy_sensor->fifo_use_flag == 0){
						if(phy_sensor->buffer != NULL){
							void* temp_ptr = phy_sensor->buffer - offsetof(struct sensor_data, data);
							bfree(temp_ptr);
							phy_sensor->buffer = NULL;
							phy_sensor->buffer_length = 0;
						}

						phy_sensor->buffer = AllocFromDss(RAW_DATA_BUFFER_LIMIT_SIZE);
						if(phy_sensor->buffer == NULL)
							break;

						phy_sensor->buffer_length = RAW_DATA_BUFFER_LIMIT_SIZE;
						phy_sensor_enable_hwfifo_with_buffer(phy_sensor->ptr, 1,
							(uint8_t *)phy_sensor->buffer, phy_sensor->buffer_length);
						phy_sensor->fifo_use_flag = 1;
					}
					break;
				}
			}
			break;
		}
	}
}

extern uint8_t raw_data_dump_flag;

static void SuspendJudge(void)
{
	for(list_t* next = feed_list.head; next != NULL; next = next->next){
		feed_general_t* feed = (feed_general_t*)next;
		if((feed->stat_flag & ON) != 0 && (feed->stat_flag & IDLE) == 0 && feed->no_idle_flag == 0){
			if(feed->idle_hold_flag == 0){
				feed->stat_flag |= IDLE;
				if(raw_data_dump_flag == 0 && feed->ctl_api.goto_idle != NULL)
					TriggerAlgoEngine(ALGO_GOTO_IDLE, (void*)feed);
					//phy_sensor ref--
				for(int i = 0; i < feed->demand_length; i++){
					sensor_data_demand_t* demand = &(feed->demand[i]);
					if(demand->freq != 0){
						sensor_handle_t* phy_sensor = GetPollSensStruct(demand->type, demand->id);
						if(phy_sensor != NULL){
							phy_sensor->idle_ref--;
							if(phy_sensor->idle_ref == 0){
								// phy_sensor IDLE
								phy_sensor->stat_flag |= IDLE;
#ifdef SUPPORT_INTERRUPT_MODE
								if(phy_sensor->fifo_length > 0 && phy_sensor->fifo_use_flag != 0){
									//if support fifo int, register cb, need_poll = 0
									if((phy_sensor->attri_mask & PHY_SENSOR_REPORT_MODE_INT_FIFO_MASK) != 0){
											phy_sensor_watermark_property_t temp_prop = {
												.count = 0,
												.callback = NULL,
												.priv_data = NULL,
												};
											phy_sensor_set_property(phy_sensor->ptr, SENSOR_PROP_FIFO_WATERMARK, &temp_prop);
											SetSensSamplingTime(phy_sensor, 0);
									}
								}else{
									phy_sensor_enable(phy_sensor->ptr, 0);
									//if support reg int, register cb, need_poll = 0
									if((phy_sensor->attri_mask & PHY_SENSOR_REPORT_MODE_INT_REG_MASK) != 0)
										phy_sensor_data_unregister_callback(phy_sensor->ptr);
								}
#endif
							}

						}
					}
				}
			}
		}
	}

	uint8_t sw_tap_detect_flag = 0;

	sensor_handle_t* tapping_sensor = GetIntSensStruct(SENSOR_TAPPING, DEFAULT_ID);
	if(tapping_sensor == NULL || (tapping_sensor->stat_flag & ON) == 0)
		sw_tap_detect_flag = 1;

	for(list_t* next = feed_list.head; next != NULL; next = next->next){
		feed_general_t* feed = (feed_general_t*)next;
		if((feed->stat_flag & ON) != 0 && feed->wake_up_clear_fifo_flag == 1)
			feed->stat_flag |= WAKE_UP_CLEAR_FIFO;

		if(sw_tap_detect_flag == 1 && feed->type == BASIC_ALGO_TAPPING && (feed->stat_flag & ON) != 0)
			feed->stat_flag |= WAKE_UP_CLEAR_FIFO;
	}

	RefleshSensorCore();

	if(sw_tap_detect_flag == 1)
		PrepareSWtappingDetect();

	for(list_t* node = phy_sensor_poll_active_list.head; node != NULL; node = node->next){
		sensor_handle_t* phy_sensor = (sensor_handle_t*)((void*)node - offsetof(sensor_handle_t, links.poll.poll_active_link));
		if((phy_sensor->stat_flag & IDLE) != 0 && phy_sensor->fifo_use_flag != 0 ){
			phy_sensor_sample_idle_t idle_prop = { .odr_x10 = phy_sensor->freq, };
			phy_sensor_set_property(phy_sensor->ptr, SENSOR_PROP_SAMPLING_IN_IDLE, &idle_prop);
		}
	}

}

static void ClearIdle(void)
{
	for(list_t* next = feed_list.head; next != NULL; next = next->next){
		feed_general_t* feed = (feed_general_t*)next;
		if((feed->stat_flag & ON) != 0){
			feed->stat_flag &= ~(WAKE_UP_CLEAR_FIFO | IDLE);
			for(int i = 0; i < feed->demand_length; i++){
				sensor_data_demand_t* demand = &(feed->demand[i]);
				if(demand->freq != 0){
					sensor_handle_t* phy_sensor = GetPollSensStruct(demand->type, demand->id);
					if(phy_sensor != NULL){
						phy_sensor->idle_ref++;
						phy_sensor->stat_flag &= ~IDLE;
					}
				}
			}
		}
	}
}

static int ReadoutFifoProcess(void)
{
	if(read_out_fifo_flag == 0){
		ClearIdle();
		RefleshSensorCore();
	}else
		TriggerAlgoEngine(CLEAR_FIFOS, NULL);
	return 0;
}

static void ResumeJudge(void)
{
	for(list_t* next = feed_list.head; next != NULL; next = next->next){
		feed_general_t* feed = (feed_general_t*)next;
		if((feed->stat_flag & IDLE) != 0){
			if(feed->ctl_api.out_idle != NULL)
				TriggerAlgoEngine(ALGO_OUT_IDLE, (void*)feed);
		}
	}

	sensor_handle_t* tapping_sensor = GetIntSensStruct(SENSOR_TAPPING, DEFAULT_ID);
	if(tapping_sensor == NULL || (tapping_sensor->stat_flag & ON) == 0){
		read_out_fifo_flag = 1;
		ReadoutFifoProcess();
	}else{
		ClearIdle();
		RefleshSensorCore();
	}
}

static struct ia_cmd* ParseCmd(struct ia_cmd* inbound)
{
	struct ia_cmd *resp = NULL;
	static struct ia_cmd* resp_save;
	switch(inbound->cmd_id){
		case CMD_GET_SENSOR_LIST:
			{
				struct sensor_type_bit_map *bit_ptr =
					(struct sensor_type_bit_map*)(inbound->param);
				uint32_t bitmap = bit_ptr->bit_map;
				resp = GetCoreSensorList(bitmap);
				if(resp != NULL)
					resp->cmd_id = RESP_GET_SENSOR_LIST;
			}
			break;
		case CMD_START_SENSOR:
			{
				struct sensor_id* param = (struct sensor_id*)inbound->param;
				resp = CoreSensorControl(param, ALGO_OPEN);
				if(resp != NULL)
					resp->cmd_id = RESP_START_SENSOR;
			}
			break;
		case CMD_STOP_SENSOR:
			{
				struct sensor_id* param = (struct sensor_id*)inbound->param;
				resp = CoreSensorControl(param, ALGO_CLOSE);
				if(resp != NULL)
					resp->cmd_id = RESP_STOP_SENSOR;
			}
			break;
		case CMD_SUBSCRIBE_SENSOR_DATA:
			{
				struct subscription* param = (struct subscription*)inbound->param;
				resp = CoreSensorControl((struct sensor_id*)param, ALGO_SUBSCRIBE);
				if(resp != NULL)
					resp->cmd_id = RESP_SUBSCRIBE_SENSOR_DATA;
			}
			break;
		case CMD_UNSUBSCRIBE_SENSOR_DATA:
			{
				struct subscription* param = (struct subscription*)inbound->param;
				resp = CoreSensorControl((struct sensor_id*)param, ALGO_UNSUBSCRIBE);
				if(resp != NULL)
					resp->cmd_id = RESP_UNSUBSCRIBE_SENSOR_DATA;
			}
			break;
		case CMD_GET_PROPERTY:
			{
				struct get_property* param = (struct get_property*)inbound->param;
				resp = CoreSensorControl((struct sensor_id*)param, ALGO_GET_PROPERTY);
				if(resp != NULL)
					resp->cmd_id = RESP_GET_PROPERTY;
			}
			break;
		case CMD_SET_PROPERTY:
			{
				struct set_property* param = (struct set_property*)inbound->param;
				resp = CoreSensorControl((struct sensor_id*)param, ALGO_SET_PROPERTY);
				if(resp != NULL)
					resp->cmd_id = RESP_SET_PROPERTY;
			}
			break;
		case CMD_CALIBRATION:
			{
				struct calibration* cali_param = (struct calibration*)inbound->param;
				switch(cali_param->clb_cmd){
					case START_CALIBRATION_CMD:
						{
							//check calibration_flag whether is 1, if 1 return error.
							//only support both a sensor and g sensor.
							//calibration_flag = 1.
							//pm_wakelock_acquire.
							//save exposed sensor stat , feed stat and feed demand array.
							//stop all the feeds but calibration target sensor.
							struct sensor_id* sensor_id = (struct sensor_id*)cali_param;
							resp = CoreSensorControl(sensor_id, ALGO_CALIBRATION_START);
							if(resp == NULL)
								break;
							resp->cmd_id = RESP_CALIBRATION;
							struct resp_calibration* resp_param = (struct resp_calibration*)resp->param;
							resp_param->clb_cmd = cali_param->clb_cmd;
							resp_param->length = 0;
						}
						break;
					case GET_CALIBRATION_CMD:
						{
							//check calibration_flag whether 0. if 0 return error
							//check no_motion_flag whether is 1, if 0 return error.
							//raw_data_calibration_flag = 1;
							//process target sensor calibration and return the result.
							struct sensor_id* sensor_id = (struct sensor_id*)cali_param;
							resp = CoreSensorControl(sensor_id, ALGO_CALIBRATION_PROCESS);
							if(resp == NULL)
								break;
							resp->cmd_id = RESP_CALIBRATION;
							struct resp_calibration* resp_param = (struct resp_calibration*)resp->param;
							resp_param->clb_cmd = cali_param->clb_cmd;
							resp_param->calibration_type = sensor_id->sensor_type;
							struct return_value* rv = (struct return_value*)resp->param;
							if(rv->ret == RESP_SUCCESS){
								sensor_handle_t* phy_sensor =
									GetPollSensStruct(sensor_id->sensor_type, sensor_id->dev_id);
								if(phy_sensor != NULL){
									resp_param->length = phy_sensor->sensor_data_frame_size;
									memset(phy_sensor->clb_data_buffer, 0,
										phy_sensor->sensor_data_frame_size);
								}else
									resp_param->length = 0;

								resp->cmd_id = inbound->cmd_id;
								resp->tran_id = inbound->tran_id;
								resp->conn_client = inbound->conn_client;
								resp->priv_data_from_client = inbound->priv_data_from_client;
								resp_save = resp;
								resp = NULL;
							}else
								resp_param->length = 0;
						}
						break;
					case STOP_CALIBRATION_CMD:
						{
							//check calibration_flag whether 0. if 0 return error
							//check raw_data_calibration_flag whether is 1 , if i return error
							//pm_wakelock_release
							//resume all the feeds previous status
							//restore exposed sensor stat, feed stat and feed demand array.
							//calibration_flag = 0;
							struct sensor_id* sensor_id = (struct sensor_id*)cali_param;
							resp = CoreSensorControl(sensor_id, ALGO_CALIBRATION_STOP);
							if(resp == NULL)
								break;
							resp->cmd_id = RESP_CALIBRATION;
							struct resp_calibration* resp_param = (struct resp_calibration*)resp->param;
							resp_param->clb_cmd = cali_param->clb_cmd;
							resp_param->length = 0;
							pm_wakelock_release(&opencore_cali_wl);
						}
						break;
					case REBOOT_AUTO_CALIBRATION_CMD:
					case SET_CALIBRATION_CMD:
						{
							struct sensor_id* sensor_id = (struct sensor_id*)cali_param;
							resp = CoreSensorControl(sensor_id, ALGO_CALIBRATION_SET);
							if(resp == NULL)
								break;
							resp->cmd_id = RESP_CALIBRATION;
							struct resp_calibration* resp_param = (struct resp_calibration*)resp->param;
							resp_param->clb_cmd = cali_param->clb_cmd;
							resp_param->length = 0;
						}
						break;
					default:
						break;
				}
			}
			break;
		case CMD_CALIBRATION_PROCESS_RESULT:
			{
				//check inbound->tran_id whether 0, if 0 calibration is succed otherwise fail.
				struct resp_calibration* param_to = (struct resp_calibration*)resp_save->param;
				if(inbound->tran_id == 0 && param_to->length > 0){
					memcpy(param_to->calib_params, inbound->param, inbound->length);
				}else if(inbound->tran_id == 1){
					struct return_value* rv = (struct return_value*)param_to;
					rv->ret = RESP_DEVICE_EXCEPTION;
				}
				resp = resp_save;
				goto cali_get_data_out;
			}
		case CMD_SUSPEND_SC:
			{
				no_motion_flag = 1;
				SuspendJudge();
			}
			break;
		case CMD_RESUME_SC:
			{
				no_motion_flag = 0;
				if(raw_data_calibration_flag == 1)
					calibration_process_error = 1;
				ResumeJudge();
			}
			break;
		case CMD_SET_HW_TAPPING_AS_WAKEUP_SOURCE:
			{
				sensor_handle_t* tapping_sensor = GetIntSensStruct(SENSOR_TAPPING, DEFAULT_ID);
				if(tapping_sensor != NULL){
					sensor_handle_t* any_motion_sensor = GetIntSensStruct(SENSOR_ANY_MOTION, DEFAULT_ID);
					if(any_motion_sensor != NULL)
						CloseIntSensor(any_motion_sensor);
					OpenIntSensor(tapping_sensor);
				}
			}
			break;
		case CMD_FIFO_READ_ONESHOT_OVER_SC:
			{
				sensor_handle_t* phy_sensor;
				memcpy(&phy_sensor, inbound->param, inbound->length);
				if(phy_sensor != NULL){
					TriggerAlgoEngine(READ_FIFO, (void*)phy_sensor);
				}
			}
			break;
		case CMD_FIFO_CLEAR_CHECK_SC:
			{
				ReadoutFifoProcess();
			}
			break;
#ifdef SUPPORT_INTERRUPT_MODE
		case CMD_RAWDATA_REG_INT_SC:
			{
				struct sensor_data* sensor_data = (struct sensor_data*)inbound->param;
				sensor_handle_t* phy_sensor = GetPollSensStruct(sensor_data->sensor.sensor_type, sensor_data->sensor.dev_id);
				if(phy_sensor != NULL && (phy_sensor->stat_flag & IDLE) == 0){
					raw_data_node_t* node = (raw_data_node_t*)AllocFromDss(sizeof(raw_data_node_t));
					if(node == NULL)
						pr_error(LOG_MODULE_OPEN_CORE, "fail to alloc raw data node");

					void* buffer = AllocFromDss(sensor_data->data_length);
					if(buffer == NULL)
						pr_error(LOG_MODULE_OPEN_CORE, "fail to alloc raw data buffer reg");

					if(node != NULL && buffer != NULL){
						node->buffer = buffer;
						node->raw_data_count = sensor_data->data_length / phy_sensor->sensor_data_frame_size;
						memcpy(buffer, sensor_data->data, sensor_data->data_length);
						list_add(&phy_sensor->raw_data_head, &node->raw_data_node);
					}else{
						if(node != NULL)
							FreeInDss((void*)node);
						if(buffer != NULL)
							FreeInDss(buffer);
					}
				}
			}
			break;
		case CMD_RAWDATA_FIFO_INT_SC:
			{
				sensor_handle_t* phy_sensor;
				memcpy(&phy_sensor, inbound->param, inbound->length);
				if(phy_sensor != NULL && (phy_sensor->stat_flag & IDLE) == 0){
					list_t* share_list_head = &phy_sensor->fifo_share_link;
					do{
						sensor_handle_t* phy_sensor_node = (sensor_handle_t*)((void*)share_list_head
							- offsetof(sensor_handle_t, fifo_share_link));
						if((phy_sensor_node->stat_flag & ON) != 0)
							TriggerAlgoEngine(READ_FIFO, (void*)phy_sensor_node);
						share_list_head = share_list_head->next;
					}while(share_list_head != &phy_sensor->fifo_share_link);
				}
			}
			break;
#endif
		default:
			break;
	}

	if(resp != NULL){
		resp->tran_id = inbound->tran_id;
		resp->conn_client = inbound->conn_client;
		resp->priv_data_from_client = inbound->priv_data_from_client;
	}

cali_get_data_out:
	bfree(inbound);
	return resp;
}

static void opencore_fiber(void)
{
	struct ia_cmd *cmd;
	struct ia_cmd *resp;
	uint32_t ts_last;
	int ret;
	int act_algo = 0;
	int queued_cmd_count = 0;

	while(1){
		ret = -2;
		ts_last = get_uptime_ms();

		if(poll_timeout > 0 || poll_timeout == OS_WAIT_FOREVER)
			ret = ipc_core_receive(&cmd, poll_timeout);

		pm_wakelock_acquire(&opencore_main_wl);

		if(ret == 0){
			cmd_node_t* cmd_node = (cmd_node_t*)balloc(sizeof(cmd_node_t), NULL);
			memset(cmd_node, 0, sizeof(cmd_node_t));
			cmd_node->cmd = cmd;
			if (queued_cmd_count < THRESHOLD_CMD_COUNT) {
				queued_cmd_count++;
			} else {
					list_t* node = list_get(&cmd_head);
					cmd_node_t* cmd_node = (cmd_node_t*)((void*)node
						- offsetof(cmd_node_t, cmd_link));
					bfree(cmd_node->cmd);
					bfree(cmd_node);
					pr_error(LOG_MODULE_OPEN_CORE, "discarding oldest command");
			}
			list_add(&cmd_head, &cmd_node->cmd_link);
		}

		if(1 == 1){
			uint32_t actual_elapse = get_uptime_ms() - ts_last;
			if(actual_elapse > poll_timeout && actual_elapse - poll_timeout > 3)
				pr_error(LOG_MODULE_OPEN_CORE, "poll=%d, elapse=%d,ret=%d", poll_timeout, actual_elapse, ret);
		}

back:
		ts_last = get_uptime_ms();
		act_algo = SensorCoreProcess(&poll_timeout);
		if(1 == 1){
			if(get_uptime_ms() - ts_last > 30)
				pr_error(LOG_MODULE_OPEN_CORE, "pt=%d, trc=%d, roc=%d, loop=%d, reg=%d, fifo=%d",
					poll_timeout, get_uptime_ms() - ts_last, read_ohrm_consume, loop, reg_mark, fifo_mark);
		}
		if(act_algo > 0)
			TriggerAlgoEngine(ALGO_PROCESS, NULL);

		ts_last = get_uptime_ms();

		if(poll_timeout == FOREVER_VALUE || poll_timeout >= 5){
			int ret = 0;
			for(list_t* node = cmd_head.head; node != NULL;){
				cmd_node_t* cmd_node = (cmd_node_t*)((void*)node
					- offsetof(cmd_node_t, cmd_link));
				if(cmd_node != NULL && cmd_node->cmd != NULL){
					resp = ParseCmd(cmd_node->cmd);
					if(resp != NULL){
						ipc_2svc_send(resp);
						bfree(resp);
					}
				}
				node = node->next;
				bfree(cmd_node);
				ret++;
				queued_cmd_count--;
			}
			memset(&cmd_head, 0, sizeof(list_head_t));
			if(ret != 0)
				goto back;
		}

		if(poll_timeout != FOREVER_VALUE){
			int uptime = get_uptime_ms();
			if(uptime - ts_last >= poll_timeout)
				goto back;
			else
				poll_timeout -= uptime - ts_last;
		}

		pm_wakelock_release(&opencore_main_wl);
	}
}

int sensor_core_create(T_QUEUE service_mgr_queue)
{
	ipc_svc_core_create();
	SensorCoreInit();
	AlgoEngineInit(service_mgr_queue);
	task_fiber_start(&stack[0], STACKSIZE, (nano_fiber_entry_t)opencore_fiber, 0, 0, 7, 0);
	return 0;
}
/* *INDENT-ON* */
