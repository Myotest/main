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
#include <stdio.h>
#include <device.h>
#include <rtc.h>
#include "os/os.h"
#include "infra/log.h"
#include "infra/tcmd/handler.h"
#include "infra/system_events.h"
#include "infra/version.h"
#include "util/misc.h"
#include "util/assert.h"

/* SPI flash management */
#include "cir_storage.h"
#include "util/cir_storage_flash_spi.h"
#include "project_mapping.h"

/* Panic management */
#include "drivers/soc_flash.h"
#include "panic_quark_se.h"

/* Time */
#include "infra/time.h"

/* TODO: Use the commonly defined constant instead */
#define PANIC_NVM_BASE (DEBUGPANIC_START_BLOCK * EMBEDDED_FLASH_BLOCK_SIZE)

/* Align address on 32bits (add 3 then clear LSBs) */
#define PANIC_ALIGN_32(x) (((uint32_t)(x) + 3) & ~(3))
#define DEFAULT_ADDRESS ~(0)
#define RTC_DRV_NAME "RTC"

DEFINE_LOG_MODULE(LOG_MODULE_SYSTEM_EVENTS, "SYEV")

static cir_storage_t * storage = NULL;

static void store_panics(uint32_t);

void system_events_init()
{
	struct device *rtc_dev;

	BUILD_BUG_ON(sizeof(struct system_event) > SYSTEM_EVENT_SIZE);

	rtc_dev = device_get_binding(RTC_DRV_NAME);
	assert(rtc_dev != NULL);
	storage = cir_storage_flash_spi_init(SYSTEM_EVENT_SIZE,
					     SPI_SYSTEM_EVENT_START_BLOCK,
					     SPI_SYSTEM_EVENT_NB_BLOCKS);

	/* Initialization of absolute time using rtc if rtc not reseted at boot */
	if (rtc_read(rtc_dev) > time())
		set_time(rtc_read(rtc_dev));

	store_panics(PANIC_NVM_BASE);
}

void __attribute__((weak)) on_system_event_generated(struct system_event *evt)
{
}

void system_event_push(struct system_event *event)
{
	if (storage) {
		memcpy(event->h.hash, version_header.hash, sizeof(event->h.hash));
		cir_storage_push(storage, (uint8_t *)event);
		on_system_event_generated(event);
	}
}

struct system_event *system_event_pop()
{
	if (storage) {
		struct system_event *evt = balloc(SYSTEM_EVENT_SIZE, NULL);
		cir_storage_err_t err = cir_storage_pop(storage, (uint8_t *)evt);
		if (err == CBUFFER_STORAGE_SUCCESS)
			return evt;
		else {
			bfree(evt);
			return NULL;
		}
	} else {
		return NULL;
	}
}

static bool prepare_crash_event_to_store(
	struct system_event *event_to_store,
	struct panic_data_flash_header *
	header,
	uint32_t
	panic_addr)
{
	unsigned int retlen;
	DRIVER_API_RC ret;
	bool event_completed = false;

	union {
		struct arcv2_panic_arch_data arc;
		struct x86_panic_arch_data x86;
	} *panic_data;
	panic_data =
		balloc(header->struct_size -
		       sizeof(struct panic_data_flash_header),
		       NULL);

	ret = soc_flash_read(panic_addr + sizeof(*header),
			     (header->struct_size -
			      sizeof(*header)) / sizeof(uint32_t),
			     &retlen,
			     (uint32_t *)panic_data);

	if (!ret && header->arch == ARC_CORE) {
		event_completed = true;
		event_to_store->event_data.panic.cpu = SYSTEM_EVENT_PANIC_ARC;
		event_to_store->event_data.panic.values[0] =
			panic_data->arc.eret;
		event_to_store->event_data.panic.values[1] =
			panic_data->arc.ecr;
		event_to_store->event_data.panic.values[2] =
			panic_data->arc.efa;
		event_to_store->h.timestamp = header->time;
	} else if (!ret) {
		event_completed = true;
		event_to_store->event_data.panic.cpu = SYSTEM_EVENT_PANIC_QUARK;
		event_to_store->event_data.panic.values[0] =
			panic_data->x86.eip;
		event_to_store->event_data.panic.values[1] =
			panic_data->x86.type;
		event_to_store->event_data.panic.values[2] =
			panic_data->x86.error;
		event_to_store->h.timestamp = header->time;
	}
	bfree(panic_data);
	return event_completed;
}

static void store_panics(uint32_t panic_addr)
{
	unsigned int retlen;
	DRIVER_API_RC ret;
	struct panic_data_flash_header *header =
		balloc(sizeof(struct panic_data_flash_header), NULL);

	/* Read flash to find panics */
	while ((soc_flash_read(panic_addr,
			       sizeof(struct panic_data_flash_header) /
			       sizeof(uint32_t),
			       &retlen,
			       (uint32_t *)header) == DRV_RC_OK)) {
		/* If no panic, stop to search panics in flash */
		if (header->magic != PANIC_DATA_MAGIC)
			break;

		/* Panic shall be sent if valid and not already pulled */
		if ((header->flags & PANIC_DATA_FLAG_FRAME_VALID) &&
		    (header->flags & PANIC_DATA_FLAG_FRAME_BLE_AVAILABLE)) {
			/* Update header to not store again panic data */
			header->flags = header->flags &
					~PANIC_DATA_FLAG_FRAME_BLE_AVAILABLE;
			/* Erase not needed because only one bit is modified from 1 to 0 */
			ret = soc_flash_write(
				panic_addr,
				sizeof(struct
				       panic_data_flash_header) /
				sizeof(uint32_t),
				&retlen,
				(uint32_t *)header);
			if (ret) {
				pr_error(LOG_MODULE_SYSTEM_EVENTS,
					 "Header update failed");
				break;
			}

			/* Store panic */
			struct system_event evt;
			evt.h.type = SYSTEM_EVENT_TYPE_PANIC;
			evt.h.size = SYSTEM_EVENT_SIZE;

			if (prepare_crash_event_to_store(&evt, header,
							 panic_addr)) {
				system_event_push(&evt);
			}
		}
		/* Here, panic magic found. Increment where ptr and start over */
		panic_addr = PANIC_ALIGN_32(panic_addr + header->struct_size);
	}

	bfree(header);
}

void system_event_fill_header(struct system_event *e, int type)
{
	e->h.type = type;
	e->h.size = SYSTEM_EVENT_SIZE;
	e->h.version = SYSTEM_EVENT_VERSION;
	e->h.timestamp = time();
}

void system_event_push_boot_event(enum reset_reasons	reset_reason,
				  enum boot_targets	boot_target)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_BOOT);
	evt.event_data.boot.reason = reset_reason;
	evt.event_data.boot.target = boot_target;
	system_event_push(&evt);
}

void system_event_push_shutdown(int type, int reason)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_SHUTDOWN);
	evt.event_data.shutdown.type = type;
	evt.event_data.shutdown.reason = reason;
	system_event_push(&evt);
}

void system_event_push_battery(uint8_t type, uint8_t data)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_BATTERY);
	evt.event_data.battery.type = type;
	evt.event_data.battery.u.data = data;
	system_event_push(&evt);
}

void system_event_push_uptime()
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_UPTIME);
	evt.event_data.uptime.time = get_uptime_ms() / 1000;

	system_event_push(&evt);
}

void system_event_push_ble_pairing(bool is_paired)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_BLE_PAIRING);
	evt.event_data.ble_pairing.is_paired = is_paired;
	system_event_push(&evt);
}

void system_event_push_ble_conn(bool is_connected, uint8_t *ble_address)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_BLE_CONN);
	evt.event_data.ble_conn.is_connected = is_connected;
	if (ble_address) {
		memcpy(evt.event_data.ble_conn.bd_address, ble_address,
		       sizeof(evt.event_data.ble_conn.bd_address));
	} else {
		memset(evt.event_data.ble_conn.bd_address, 0,
		       sizeof(evt.event_data.ble_conn.bd_address));
	}
	system_event_push(&evt);
}

void system_event_push_worn(bool data)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_WORN);
	evt.event_data.worn.is_worn = (uint8_t)data;
	system_event_push(&evt);
}

void system_event_push_nfc_reader_detected(bool data)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_NFC);
	evt.event_data.nfc.is_active = (uint8_t)data;
	system_event_push(&evt);
}

void system_event_push_button(uint8_t button_id, uint8_t event, uint32_t param)
{
	struct system_event evt;

	system_event_fill_header(&evt, SYSTEM_EVENT_TYPE_BUTTON);
	evt.event_data.button.button_id = button_id;
	evt.event_data.button.event = event;
	evt.event_data.button.param = param;
	system_event_push(&evt);
}


#ifdef CONFIG_SYSTEM_EVENTS_TCMD
void __attribute__((weak)) project_dump_event(struct system_event *	event,
					      struct tcmd_handler_ctx * ctx)
{
}

void dump_events(int argc, char **argv, struct tcmd_handler_ctx *ctx)
{
	static const char *type_string[] = {
		"",
		"BOOT",
		"PANIC",
		"SHUTDOWN",
		"UPTIME",
		"BATTERY",
		"BLE_PAIRING",
		"BLE_CONN",
		"WORN",
		"NFC_READER",
		"BUTTON",
	};

	struct system_event *evt;

	do {
		evt = system_event_pop();
		if (evt) {
			if (evt->h.type < SYSTEM_EVENT_USER_RANGE_START) {
#define TMP_BUF_SZ 80
				/* *INDENT-OFF* */
				char buf[TMP_BUF_SZ];
				snprintf(buf, TMP_BUF_SZ, "%d [%x%x%x%x]: %s", evt->h.timestamp,
						evt->h.hash[0], evt->h.hash[1], evt->h.hash[2],
						evt->h.hash[3], type_string[evt->h.type]);
				char * append = buf + strlen(buf);
				switch (evt->h.type) {
					case SYSTEM_EVENT_TYPE_BATTERY:
					if (evt->event_data.battery.type == SYSTEM_EVENT_BATT_CHARGE) {
						static const char *batt_state[] = {
								"CHARGING",
								"DISCHARGING",
								"COMPLETE"};
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" state: %s",
								batt_state[evt->event_data.battery.u.state-1]);
					} else {
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" level: %d", evt->event_data.battery.u.data);
					}
					break;
					case SYSTEM_EVENT_TYPE_BOOT:
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" reason: %d", evt->event_data.boot.reason);
					break;
					case SYSTEM_EVENT_TYPE_PANIC:
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" [%s] ", evt->event_data.panic.cpu ==
										SYSTEM_EVENT_PANIC_ARC ? "ARC" : "QRK");
					break;
					case SYSTEM_EVENT_TYPE_UPTIME:
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" time: %d s", evt->event_data.uptime.time);
					break;
					case SYSTEM_EVENT_TYPE_BLE_PAIRING:
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" paired: %d",
								evt->event_data.ble_pairing.is_paired);
					break;
					case SYSTEM_EVENT_TYPE_BLE_CONN:
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" connected: %d , ble_address %02x:%02x:%02x:%02x:%02x:%02x",
								evt->event_data.ble_conn.is_connected,
								evt->event_data.ble_conn.bd_address[5],
								evt->event_data.ble_conn.bd_address[4],
								evt->event_data.ble_conn.bd_address[3],
								evt->event_data.ble_conn.bd_address[2],
								evt->event_data.ble_conn.bd_address[1],
								evt->event_data.ble_conn.bd_address[0]);
					break;
					case SYSTEM_EVENT_TYPE_WORN:
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" worn: %d", evt->event_data.worn.is_worn);
					break;
					case SYSTEM_EVENT_TYPE_NFC:
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" nfc reader: %d", evt->event_data.nfc.is_active);
					break;
					case SYSTEM_EVENT_TYPE_BUTTON:
						snprintf(append, TMP_BUF_SZ-(append-buf),
								" button: %d event [%x, %d]",
								evt->event_data.button.button_id,
								evt->event_data.button.event,
								evt->event_data.button.param);
					break;
				}
				TCMD_RSP_PROVISIONAL(ctx, buf);
				/* *INDENT-ON* */
			} else {
				project_dump_event(evt, ctx);
			}
			bfree(evt);
		}
	} while (evt);
	TCMD_RSP_FINAL(ctx, NULL);
}

DECLARE_TEST_COMMAND(system, dump_evt, dump_events);
#endif
