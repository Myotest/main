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

#include <misc/util.h>
#include "util/compiler.h"

#include "infra/log.h"
#include "infra/panic.h"
#include "infra/port.h"

#include "services/services_ids.h"
#include "services/ui_service/ui_service.h"
#include "ui_service_private.h"

#include "drivers/led/led.h"
#include "drivers/haptic.h"
#include "lib/button/button.h"

#if defined(CONFIG_UI_SERVICE_IMPL_BUTTON) && defined(CONFIG_BUTTON_FEEDBACK)
#include "util/workqueue.h"
#endif
#ifdef CONFIG_SYSTEM_EVENTS
#include "infra/system_events.h"
#endif

/* FIXME UI service should include a user config header which resolves
 * the right led/haptic driver header */
#ifdef CONFIG_LP5562_LED
#include "lib/led/lp5562_led.h"
#endif
#ifdef CONFIG_SOC_LED
#include "lib/led/soc_led.h"
#endif
#ifdef CONFIG_GPIO_LED
#include "lib/led/gpio_led.h"
#endif

#define PANIC_FAIL_1 1
#define PANIC_FAIL_2 2

#define UI_SHUTDOWN_WAIT_MS 1000

static uint32_t ui_available_events;
static uint32_t ui_available_events_enabled;
static uint32_t ui_available_notifications;

/* Track state of ui service while treating request to set events. */
static uint8_t ui_svc_handler_state;

void *ui_svc_queue_msg;
ui_events_req_t ui_events_req;

#ifdef CONFIG_UI_SERVICE_IMPL_LED
typedef struct ui_led_evt {
	struct message m;
	uint8_t led_id;
	uint8_t event;
} ui_led_evt_t;

/* UI led state for race condition protection */
enum {
	UI_LED_IDLE, /*!< LED stopped or playing background pattern */
	UI_LED_DONE, /*!< LED pattern just finished */
	UI_LED_PLAYING, /*!< LED pattern playing */
};
static struct cfw_message *ui_led_reqs[UI_LED_COUNT] = { NULL };
#ifdef CONFIG_LED_MULTICOLOR
static rgb_t ui_led_default_color[UI_LED_COUNT];
#endif
static bool ui_led_is_default_color[UI_LED_COUNT] = { false };
static volatile uint8_t ui_led_state[UI_LED_COUNT];
static struct cfw_message *ui_led_queue[UI_LED_COUNT] = { NULL };
#endif

static struct cfw_message *ui_vibr_req;

extern struct ui_service_config ui_config;

static T_SEMAPHORE led_play_done = NULL;
static T_SEMAPHORE vibr_play_done = NULL;

/* Initialize list of event_id + associated service/client pairs. */
/* To be completed. */
ui_events_interfaces_list_t ui_events_interfaces_list[MAX_SERVICES] = {
#ifdef CONFIG_UI_SERVICE_IMPL_BUTTON
	{ UI_BTN_SINGLE_EVT_IDX, UIDRV_BTN_BASE },
	{ UI_BTN_DOUBLE_EVT_IDX, UIDRV_BTN_BASE },
	{ UI_BTN_MAX_EVT_IDX, UIDRV_BTN_BASE },
	{ UI_BTN_MULTIPLE_EVT_IDX, UIDRV_BTN_BASE },
#endif
#ifdef TEST_UI_SVC
	{ UI_TEST_EVT_IDX, TEST_SVC_ID },
#endif
};

extern void ui_init_client_service_table(void);
extern int8_t ui_handle_evt_req_for_service(uint8_t svc_id);
extern int8_t ui_handle_evt_req_for_driver(uint16_t drv_id, uint32_t evt,
					   uint32_t enabled);

static void ui_handle_feature_req(struct cfw_message *msg);
static void ui_handle_get_enabled_event_req(struct cfw_message *msg);
static void ui_handle_set_event_req(struct cfw_message *msg);

#ifdef CONFIG_UI_SERVICE_IMPL_LED
static void ui_play_led(struct cfw_message *msg);
static void ui_process_led_event(struct cfw_message *msg);
void ui_play_led_callback(uint8_t led_id, uint8_t result);
#endif
#ifdef CONFIG_UI_SERVICE_IMPL_HAPTIC
static struct device *haptic_dev;
static void ui_play_vibr(struct cfw_message *msg);
#endif
static void ui_handle_message(struct cfw_message *msg, void *param);
void ui_handle_events_callback(uint32_t item, uint32_t param);

void ui_client_disconnected(conn_handle_t *instance);
void ui_client_connected(conn_handle_t *instance);
#ifdef CONFIG_UI_SERVICE_IMPL_BUTTON
static void ui_button_notification_callback(uint8_t button_id, uint8_t event,
					    uint32_t param);
#endif
static void ui_service_shutdown(service_t *svc, struct cfw_message *msg);

static service_t ui_service = {
	.service_id = UI_SVC_SERVICE_ID,
	.client_connected = ui_client_connected,
	.client_disconnected = ui_client_disconnected,
	.shutdown_request = ui_service_shutdown,
};

#define MSG_ID_UI_SERVICE_SHUTDOWN_REQ 0xff00

static void ui_service_shutdown(service_t *svc, struct cfw_message *msg)
{
	struct cfw_message *sm = (struct cfw_message *)message_alloc(
		sizeof(*sm), NULL);

	/* In order to ensure that any pending requests are processed prior to the
	 * service shutdown, send a message to self so it will be processed after
	 * all other pending requests.
	 */
	CFW_MESSAGE_ID(sm) = MSG_ID_UI_SERVICE_SHUTDOWN_REQ;
	CFW_MESSAGE_DST(sm) = ui_service.port_id;
	CFW_MESSAGE_SRC(sm) = ui_service.port_id;
	CFW_MESSAGE_PRIV(sm) = msg;
	cfw_send_message(sm);
}

void ui_client_connected(conn_handle_t *instance)
{
	pr_debug(LOG_MODULE_UI_SVC,
		 "UI service connected, client port:%d svc id:%d %d",
		 instance->client_port,
		 instance->svc->service_id, ui_service.service_id);
}

void ui_client_disconnected(conn_handle_t *instance)
{
	pr_debug(LOG_MODULE_UI_SVC,
		 "UI service disconnected, client port:%d svc id:%d",
		 instance->client_port,
		 instance->svc->service_id);
}

/*
 * Function to handle messages coming from the framework.
 *
 * @details The function gets the received message and executes
 *          the function corresponding to the received message id.
 * @param[in]  msg   Pointer to message.
 * @param[in]  param Additional data. NULL if not used.
 * @return   none
 */
static void ui_handle_message(struct cfw_message *msg, void *param)
{
	if (msg == NULL)
		panic(PANIC_FAIL_1);

	pr_debug(LOG_MODULE_UI_SVC,
		 "UI service received msg: %X src: %d dst: %d",
		 CFW_MESSAGE_ID(msg),
		 CFW_MESSAGE_SRC(msg),
		 CFW_MESSAGE_DST(msg));

	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_UI_GET_FEAT_REQ:
		ui_handle_feature_req(msg);
		break;
	case MSG_ID_UI_GET_EVT_REQ:
		ui_handle_get_enabled_event_req(msg);
		break;
	case MSG_ID_UI_SET_EVT_REQ:
		ui_handle_set_event_req(msg);
		break;
#ifdef CONFIG_UI_SERVICE_IMPL_LED
	case MSG_ID_UI_LED_REQ:
		ui_play_led(msg);
		break;
	case MSG_ID_UI_LOCAL_LED_EVT:
		ui_process_led_event(msg);
		break;
#endif
#ifdef CONFIG_UI_SERVICE_IMPL_HAPTIC
	case MSG_ID_UI_VIBR_REQ:
		ui_play_vibr(msg);
		break;
#endif
	case MSG_ID_UI_SERVICE_SHUTDOWN_REQ:
		/* Try to acquire the semaphore in order to wait
		 * if any play request are pending before shutdown.
		 */
		if (semaphore_get_count(led_play_done, NULL) <= 0)
			semaphore_take(led_play_done, UI_SHUTDOWN_WAIT_MS);
		if (semaphore_get_count(vibr_play_done, NULL) <= 0)
			semaphore_take(vibr_play_done, UI_SHUTDOWN_WAIT_MS);
		cfw_send_message(CFW_MESSAGE_PRIV(msg));
		break;
	default:
		break;
	}

	/* Free received message. */
	cfw_msg_free(msg);
}


/*
 * Callback to manage results of requests for setting events.
 *
 * @details Each time this function is called, the response is built
 *          with the event id and its state (event has been correctly
 *          enabled/disabled). If all events have been treated, it is
 *          checked if all have been enabled/disabled as expected, and
 *          completed response is sent back.
 *          Result of event(s) settings is also broadcasted to all
 *          clients.
 * @param[in]  item  Event identifier.
 * @param[in]  param Final state of the event (enabled/disabled).
 * @return   none
 */
void ui_handle_events_callback(uint32_t item, uint32_t param)
{
	int8_t status;
	struct cfw_message *evt_msg;
	ui_service_set_events_rsp_t *resp;

	if (TEST_EVT(ui_events_req.events_req, item)) {
		/* Store event which has been treated. */
		ui_events_req.events_resp |= item;

		/* Store final enabled/disabled status for this event. */
		if (param) {
			SET_EVT_BIT(ui_events_req.events_enable_resp, item);
			SET_EVT_BIT(ui_available_events_enabled, item);
		} else {
			CLEAR_EVT_BIT(ui_events_req.events_enable_resp, item);
			CLEAR_EVT_BIT(ui_available_events_enabled, item);
		}

		if (ui_events_req.events_resp == ui_events_req.events_req) {
			/* All events treated. Build answer. */
			if (ui_events_req.events_enable_resp !=
			    ui_events_req.events_enable_req) {
				pr_debug(
					LOG_MODULE_UI_SVC,
					"Some events could not be set: req mask:0x%X result:0x%X",
					ui_events_req.events_enable_req,
					ui_events_req.events_enable_resp);
				/* Some events could not be enabled/disabled. */
				status = -1;
			} else
				status = 0;

			resp = (ui_service_set_events_rsp_t *)cfw_alloc_rsp_msg(
				ui_events_req.msg_req,
				MSG_ID_UI_SERVICE_SET_ENABLED_EVENT_RSP,
				sizeof(*resp));
			if (resp == NULL) panic(PANIC_FAIL_1);

			resp->status = status;
			pr_debug(
				LOG_MODULE_UI_SVC,
				"Send response MSG_ID_UI_SERVICE_SET_ENABLED_EVENT_RSP, status:0x%X",
				status);
			cfw_send_message(resp);

			/* Config has changed. Send information to all clients. */
			evt_msg = cfw_alloc_evt_msg(&ui_service,
						    ui_available_events_enabled,
						    sizeof(*evt_msg));
			if (evt_msg == NULL) panic(PANIC_FAIL_2);
			pr_debug(
				LOG_MODULE_UI_SVC,
				"Send event to client: port:0x%X, service_id: %d, event: 0x%X",
				ui_service.port_id, ui_service.service_id,
				ui_available_events_enabled);
			cfw_send_event(evt_msg);

			/* Message cloned by the framework. Can be deleted now. */
			cfw_msg_free(evt_msg);

			/* Ready for next event(s) message request. */
			ui_events_req.events_resp = 0;
			ui_events_req.events_enable_resp = 0;
			cfw_msg_free(ui_events_req.msg_req);
			ui_events_req.msg_req = NULL;

			ui_svc_handler_state = UI_SET_EVT_READY;
		}
	}
}

/*
 * Callback to return result of played vibration pattern.
 *
 * @details Called directly by ui_play_vibr() in case the DRV2605 haptic driver is unable to
 *          play vibration, or called by the DRV2605 driver once it has finished
 *          playing the vibration.
 *          Pointer to this callback has been provided to the driver at
 *          init of the service.
 * @param[in]  result Result of played LED. I2C_WRITE_OK if success, else an
 *                    error code.
 * @return   none
 */
void ui_play_vibr_callback(int8_t result)
{
	if (ui_vibr_req == NULL) {
		pr_error(LOG_MODULE_UI_SVC, "vibra req NULL");
		return;
	}
	ui_service_play_vibr_pattern_rsp_t *resp;
	resp = (ui_service_play_vibr_pattern_rsp_t *)cfw_alloc_rsp_msg(
		ui_vibr_req,
		MSG_ID_UI_SERVICE_PLAY_VIBR_PATTERN_RSP,
		sizeof(*
		       resp));
	if (resp == NULL) panic(PANIC_FAIL_1);

	resp->status = result;

	cfw_send_message(resp);

	cfw_msg_free(ui_vibr_req);
	ui_vibr_req = NULL;

	/* Release the semaphore to continue with shutdown sequence */
	semaphore_give(vibr_play_done, NULL);
}


/*
 * Callback to return button events.
 *
 * Called directly by the button module in case of press event.
 * Pointer to this callback has been provided to the module at
 * init of the service.
 */
#ifdef CONFIG_UI_SERVICE_IMPL_BUTTON
#ifdef CONFIG_BUTTON_FEEDBACK
static vibration_u vib_push_btn = {
	.square_x2.amplitude = 100,
	.square_x2.duration_on_1 = 50,
	.square_x2.duration_off_1 = 0,
	.square_x2.duration_on_2 = 0,
	.square_x2.duration_off_2 = 0,
	.square_x2.repetition_count = 1
};

static void haptic_feedback()
{
	haptic_play(haptic_dev, VIBRATION_SQUARE_X2, &vib_push_btn);
}
#endif

static void ui_button_notification_callback(uint8_t button_id, uint8_t event,
					    uint32_t param)
{
	pr_debug(LOG_MODULE_UI_SVC,
		 "button %d event id [0x%x, %x] from btn module",
		 button_id, event, param);

#ifdef CONFIG_BUTTON_FEEDBACK
	if (event & FEEDBACK_EVT) {
		workqueue_queue_work(haptic_feedback, NULL);
		return;
	}
#endif

	union ui_service_drv_evt *broadcast_evt;
	/* Message id set to 0 here. Correctly set next lines.             */
	broadcast_evt = (union ui_service_drv_evt *)cfw_alloc_evt_msg(
		&ui_service, 0, sizeof(*broadcast_evt));
	if (broadcast_evt == NULL) panic(PANIC_FAIL_1);

	if ((event & SINGLE_PRESS) &&
	    TEST_EVT(ui_available_events, UI_BTN_SINGLE_EVT_IDX)) {
		broadcast_evt->btn_evt.header.m.id =
			MSG_ID_UI_SERVICE_BTN_SINGLE_EVT;
		goto send_event;
	} else if ((event & DOUBLE_PRESS) &&
		   TEST_EVT(ui_available_events, UI_BTN_DOUBLE_EVT_IDX)) {
		broadcast_evt->btn_evt.header.m.id =
			MSG_ID_UI_SERVICE_BTN_DOUBLE_EVT;
		goto send_event;
	} else if ((event & MAX_PRESS) &&
		   TEST_EVT(ui_available_events, UI_BTN_MAX_EVT_IDX)) {
		broadcast_evt->btn_evt.header.m.id =
			MSG_ID_UI_SERVICE_BTN_MAX_EVT;
		goto send_event;
	}
#ifdef CONFIG_BUTTON_MULTIPLE_PRESS
	else if ((event & MULTIPLE_PRESS) &&
		 TEST_EVT(ui_available_events, UI_BTN_MULTIPLE_EVT_IDX)) {
		broadcast_evt->btn_evt.header.m.id =
			MSG_ID_UI_SERVICE_BTN_MULTIPLE_EVT;
		goto send_event;
	}
#endif
	cfw_msg_free((struct cfw_message *)broadcast_evt);
	pr_error(LOG_MODULE_UI_SVC,
		 "Unknown event [0x%x] from btn module",
		 event);
	return;

send_event:
	broadcast_evt->btn_evt.btn = button_id;
	broadcast_evt->btn_evt.param = param;
	broadcast_evt->btn_evt.header.m.len = sizeof(*broadcast_evt);
#ifdef CONFIG_SYSTEM_EVENTS
	system_event_push_button(button_id, event, param);
#endif
	cfw_send_event((struct cfw_message *)broadcast_evt);
	/* Message cloned by the framework. Can be deleted now. */
	cfw_msg_free((struct cfw_message *)broadcast_evt);
}
#endif


#ifdef CONFIG_UI_SERVICE_IMPL_LED
/*
 * Function to play LED, on request received from other service.
 *
 * @details LED type, config, and duration are transmitted to the
 *          LED driver.
 * @param[in]  msg  Request message.
 * @return   none
 */
void ui_play_led(struct cfw_message *msg)
{
	uint8_t status;
	struct ui_service_play_led_pattern_rsp *resp;
	ui_play_led_pattern_req_t __maybe_unused *led_req =
		(ui_play_led_pattern_req_t *)msg;

	/* check pattern content */
	if ((led_req->type == LED_WAVE_X1 &&
	     led_req->pattern.duration[0].duration_on == 0) ||
	    ((led_req->type == LED_BLINK_X1 || led_req->type == LED_WAVE_X1) &&
	     (led_req->pattern.duration[0].duration_off == 0 &&
	      led_req->pattern.repetition_count > 1)) ||
	    ((led_req->type == LED_BLINK_X2 || led_req->type == LED_WAVE_X2) &&
	     (led_req->pattern.duration[0].duration_on == 0 ||
	      led_req->pattern.duration[0].duration_off == 0 ||
	      led_req->pattern.duration[1].duration_on == 0 ||
	      (led_req->pattern.duration[1].duration_off == 0 &&
	       led_req->pattern.repetition_count > 1))) ||
	    (led_req->type == LED_BLINK_X3 &&
	     (led_req->pattern.duration[0].duration_on == 0 ||
	      led_req->pattern.duration[0].duration_off == 0 ||
	      led_req->pattern.duration[1].duration_on == 0 ||
	      led_req->pattern.duration[1].duration_off == 0 ||
	      led_req->pattern.duration[2].duration_on == 0 ||
	      (led_req->pattern.duration[2].duration_off == 0 &&
	       led_req->pattern.repetition_count > 1)))) {
		status = UI_INVALID_PARAM;
		goto send_response;
	}

	if ((led_req->type == LED_BLINK_X1) &&
	    (led_req->pattern.duration[0].duration_on ==
	     LED_FIXED_INFINITE_DURATION)) {
		/* default pattern configuration */
		ui_led_is_default_color[led_req->led_id] = true;
#ifdef CONFIG_LED_MULTICOLOR
		ui_led_default_color[led_req->led_id] = led_req->pattern.rgb[0];
#endif

		if (ui_led_state[led_req->led_id] > UI_LED_IDLE) {
			/* Pattern already playing, do nothing */
			status = UI_SUCCESS;
		} else {
			/* LED is stopped, start the default pattern */
			status = led_pattern_handler_config(led_req->type,
							    &led_req->pattern,
							    led_req->led_id);
		}
		/* Send response to the user */
		goto send_response;
	}

	if (ui_led_state[led_req->led_id] > UI_LED_IDLE) {
		/* LED pattern still playing */
		switch (led_req->policy) {
		case PATTERN_OVERRIDE:;
			/* Stop current request */
			struct ui_service_play_led_pattern_rsp *resp;
			resp = (struct ui_service_play_led_pattern_rsp *)
			       cfw_alloc_rsp_msg(
				ui_led_reqs[led_req->led_id],
				MSG_ID_UI_SERVICE_PLAY_LED_PATTERN_RSP,
				sizeof(*resp));

			if (ui_led_state[led_req->led_id] == UI_LED_DONE) {
				/* Pattern just finished */
				resp->status = UI_SUCCESS;
			} else {
				resp->status = UI_ABORT;
			}
			resp->led_id = led_req->led_id;

			cfw_send_message(resp);
			cfw_msg_free(ui_led_reqs[led_req->led_id]);
			ui_led_state[led_req->led_id] = UI_LED_IDLE;

			/* Stop the LED to be sure the callback function is not called */
			led_pattern_handler_config(LED_NONE, NULL,
						   led_req->led_id);
			/* Release the semaphore as callback will not be called and ignore
			 * error in case of multiple releases.
			 */
			OS_ERR_TYPE err = 0;
			semaphore_give(led_play_done, &err);
			break;
		case PATTERN_CANCEL:
			status = UI_BUSY;
			goto send_response;
			break;
		case PATTERN_QUEUE:
			/* Check if a pattern is already queued */
			if (ui_led_queue[led_req->led_id]) {
				status = UI_BUSY;
				goto send_response;
			} else {
				ui_led_queue[led_req->led_id] =
					cfw_clone_message(msg);
				return;
			}
			break;
		}
	}

	if (led_req->type == LED_NONE) {
		/* At this point we are in IDLE state, with ui_led_reqs NULL */
		ui_led_is_default_color[led_req->led_id] = false;

		/* Transmit to LED driver. */
		status = led_pattern_handler_config(LED_NONE,
						    &led_req->pattern,
						    led_req->led_id);

		/* Send response to the user */
		goto send_response;
	}

	ui_led_reqs[led_req->led_id] = cfw_clone_message(msg);

	ui_led_state[led_req->led_id] = UI_LED_PLAYING;

	/* Transmit to LED driver */
	status = led_pattern_handler_config(led_req->type,
					    &led_req->pattern, led_req->led_id);

	if (status == 0) {
		/* Acquire semphore to sync up during shutdown */
		semaphore_take(led_play_done, UI_SHUTDOWN_WAIT_MS);
		return;
	}

send_response:
	resp = (struct ui_service_play_led_pattern_rsp *)
	       cfw_alloc_rsp_msg(msg,
				 MSG_ID_UI_SERVICE_PLAY_LED_PATTERN_RSP,
				 sizeof(*resp));

	/* Transmit to LED driver. */
	resp->status = status;
	resp->led_id = led_req->led_id;

	/* Acquire semphore to sync up during shutdown */
	semaphore_take(led_play_done, UI_SHUTDOWN_WAIT_MS);

	cfw_send_message(resp);
}

static void ui_process_led_event(struct cfw_message *msg)
{
	struct ui_led_evt *evt = (struct ui_led_evt *)msg;

	if (ui_led_state[evt->led_id] != UI_LED_DONE) {
		return;
	}
	/* LED pattern is done, goto idle state */
	ui_led_state[evt->led_id] = UI_LED_IDLE;

	if (!ui_led_reqs[evt->led_id]) {
		/* panic, this case should never happen */
		panic(-1);
	}

	/* Send notification for current request ending */
	struct ui_service_play_led_pattern_rsp *resp;
	resp = (struct ui_service_play_led_pattern_rsp *)
	       cfw_alloc_rsp_msg(ui_led_reqs[evt->led_id],
				 MSG_ID_UI_SERVICE_PLAY_LED_PATTERN_RSP,
				 sizeof(*resp));

	resp->status = UI_SUCCESS;
	resp->led_id = evt->led_id;

	cfw_send_message(resp);

	cfw_msg_free(ui_led_reqs[evt->led_id]);

	/* Check if a pattern is queued */
	if (ui_led_queue[evt->led_id]) {
		ui_play_led(ui_led_queue[evt->led_id]);
		cfw_msg_free(ui_led_queue[evt->led_id]);
		ui_led_queue[evt->led_id] = NULL;
	} else if (ui_led_is_default_color[evt->led_id]) {
		/* Set background pattern */
		struct led_s default_pattern;
#ifdef CONFIG_LED_MULTICOLOR
		default_pattern.rgb[0] = ui_led_default_color[evt->led_id];
#endif
		default_pattern.duration[0].duration_on =
			LED_FIXED_INFINITE_DURATION;
		led_pattern_handler_config(LED_BLINK_X1,
					   &default_pattern, evt->led_id);
	} else {
		/* Stop LED */
		led_pattern_handler_config(LED_NONE, NULL, evt->led_id);
	}
}

/*
 * Callback to return result of played LED.
 *
 * @details Called directly by ui_play_led() in case LED driver is unable to
 *          play LED, or called by the LED driver once it has finished
 *          playing the LED.
 *          Pointer to this callback has been provided to the driver at
 *          init of the service.
 * @param[in]  result Result of played LED. I2C_WRITE_OK if success, else an
 *                    error code.
 * @return   none
 */
void ui_play_led_callback(uint8_t led_id, uint8_t result)
{
	/* No need to lock IRQs as this context as
	 * the highest priority regarding the UI LED */
	if (ui_led_state[led_id] == UI_LED_PLAYING) {
		/* LED pattern is done, goto idle state */
		ui_led_state[led_id] = UI_LED_DONE;

		struct ui_led_evt *evt =
			(struct ui_led_evt *)message_alloc(sizeof(*evt), NULL);
		MESSAGE_ID(&evt->m) = MSG_ID_UI_LOCAL_LED_EVT;
		MESSAGE_DST(&evt->m) = ui_service.port_id;
		MESSAGE_SRC(&evt->m) = ui_service.port_id;
		evt->led_id = led_id;
		evt->event = result;
		port_send_message(&evt->m);

		/* Release the semaphore to continue with shutdown sequence */
		semaphore_give(led_play_done, NULL);
	}
}
#endif

#ifdef CONFIG_UI_SERVICE_IMPL_HAPTIC
/*
 * Function to play vibration pattern, on request received from other service.
 *
 * @details config and duration are transmitted to the DRV2605 haptic driver.
 * @param[in]  msg  Request message.
 * @return   none
 */
void ui_play_vibr(struct cfw_message *msg)
{
	uint8_t status = UI_SUCCESS;
	ui_play_vibr_pattern_req_t __maybe_unused *vibr_req;

	if (ui_vibr_req)
		/* Still playing a vibration pattern .                   */
		/* Reject new request: only one DRV2605 device available */
		status = UI_BUSY;
	else {
		vibr_req = (ui_play_vibr_pattern_req_t *)msg;
		/* check pattern content */
		if (vibr_req->type == VIBRATION_SQUARE_X2 &&
		    (vibr_req->pattern.square_x2.duration_on_1 == 0 ||
		     (vibr_req->pattern.square_x2.duration_off_1 == 0 &&
		      (vibr_req->pattern.square_x2.duration_on_2 != 0 ||
		       vibr_req->pattern.square_x2.repetition_count > 1)) ||
		     (vibr_req->pattern.square_x2.duration_on_2 == 0 &&
		      vibr_req->pattern.square_x2.duration_off_2 != 0) ||
		     (vibr_req->pattern.square_x2.duration_off_2 == 0 &&
		      vibr_req->pattern.square_x2.repetition_count > 1)))
			status = UI_INVALID_PARAM;
	}

	if (status != UI_SUCCESS) {
		/* cannot play pattern, return an error */
		ui_service_play_vibr_pattern_rsp_t *resp;
		resp = (ui_service_play_vibr_pattern_rsp_t *)cfw_alloc_rsp_msg(
			msg,
			MSG_ID_UI_SERVICE_PLAY_VIBR_PATTERN_RSP,
			sizeof(
				*resp));
		if (resp == NULL)
			panic(PANIC_FAIL_1);

		resp->status = status;
		cfw_send_message(resp);

		return;
	}

	ui_vibr_req = cfw_clone_message(msg);

	vibr_req = (ui_play_vibr_pattern_req_t *)ui_vibr_req;

	if (ui_vibr_req == NULL) panic(PANIC_FAIL_2);

	/* Transmit to DRV2605 haptic driver.
	 * Result will be returned in above vibr callback.
	 */
	pr_debug(LOG_MODULE_UI_SVC, "UI service : ui_play_vibr");
	haptic_play(haptic_dev, vibr_req->type, &vibr_req->pattern);

	/* Acquire semphore to sync up during shutdown */
	semaphore_take(vibr_play_done, UI_SHUTDOWN_WAIT_MS);
}
#endif

/*
 * Function to handle requests for setting events.
 *
 * @details Event mask and message are stored for future response to the request.
 *          For each event id in the mask it is verified if it is already set.
 *          If not, it is passed to function responsible for managing it with the
 *          destinator (service/driver).
 *          In case of error, reponse with failed status is sent here.
 * @param[in]  msg  Event message.
 * @return   none
 */
void ui_handle_set_event_req(struct cfw_message *msg)
{
	uint32_t event_mask;
	uint32_t enable;
	int8_t ret = 0;
	uint32_t ui_evt_idx;
	int8_t idx;
	ui_service_set_events_rsp_t *resp;
	uint8_t is_already_set = 0;
	uint8_t enable_event;

	if (ui_svc_handler_state == UI_SET_EVT_READY) {
		event_mask = ((ui_set_events_req_t *)msg)->mask;
		enable = ((ui_set_events_req_t *)msg)->enable;

		if ((ui_available_events & event_mask) > 0) {
			ui_events_req.events_req = event_mask;
			ui_events_req.events_enable_req = enable;
			ui_events_req.events_resp = 0;
			ui_events_req.events_enable_resp = 0;
			ui_events_req.msg_req = cfw_clone_message(msg);

			ui_svc_handler_state = UI_SET_EVT_BLOCKING;

			for (idx = 0;
			     idx < ARRAY_SIZE(ui_events_interfaces_list);
			     idx++) {
				if (TEST_EVT(ui_events_interfaces_list[idx].
					     event_id, event_mask)) {
					ui_evt_idx =
						ui_events_interfaces_list[idx].
						event_id;

					/* Test if event already set. */
					is_already_set = 0;
					if ((ui_events_req.events_enable_req &
					     ui_evt_idx) > 0) {
						if (TEST_EVT(
							    ui_available_events_enabled,
							    ui_evt_idx) > 0)
							/* Already enabled. */
							is_already_set = 1;
						enable_event = 1;
					} else {
						if (TEST_EVT(
							    ui_available_events_enabled,
							    ui_evt_idx) == 0)
							/* Already disabled. */
							is_already_set = 1;
						enable_event = 0;
					}

					/* Send. */
					if (!is_already_set) {
						if (ui_events_interfaces_list[
							    idx].interface_id &
						    UI_DRV_ID_BASE)
							ret =
								ui_handle_evt_req_for_driver(
									ui_events_interfaces_list
									[idx].
									interface_id,
									ui_events_interfaces_list
									[idx].
									event_id,
									enable_event);
						else
							ret =
								ui_handle_evt_req_for_service(
									ui_events_interfaces_list
									[idx].
									interface_id);
					} else {
						/* Event was already set. Start filling the answer */
						/* before examining next event in mask.            */
						ui_handle_events_callback(
							ui_events_interfaces_list
							[idx].
							event_id,
							enable_event);
					}
				}
			}
		} else {
			/* Invalid passed mask. */
			ret = UI_INVALID_PARAM;
		}
	} else {
		/* Another set event(s) request is ongoing. */
		ret = UI_BUSY;
	}

	if (ret) {
		/* Request could not be treated (error or service busy, */
		/* or already done or partially done.                   */
		/* Send back status.                                    */
		if (ret == UI_BUSY) {
			pr_debug(
				LOG_MODULE_UI_SVC,
				"Warning: UI service busy with another request");
		} else if (ret == UI_INVALID_PARAM) {
			pr_debug(LOG_MODULE_UI_SVC,
				 "UI event handle: unknown event mask 0x%X",
				 event_mask);
		} else {
			pr_debug(
				LOG_MODULE_UI_SVC,
				"Some events could not be set: evt req mask:0x%X treated:0x%X",
				ui_events_req.events_req,
				ui_events_req.events_resp);
		}

		resp = (ui_service_set_events_rsp_t *)cfw_alloc_rsp_msg(
			msg, MSG_ID_UI_SERVICE_SET_ENABLED_EVENT_RSP,
			sizeof(*resp));
		if (resp == NULL) panic(PANIC_FAIL_1);

		resp->status = ret;
		cfw_send_message(resp);

		ui_svc_handler_state = UI_SET_EVT_READY;
	}
}

/*
 * Function to handle requests to get current events and notifications.
 *
 * @details Reponse build with the current available events and notifications
 *          and sent back via the framework.
 * @param[in]  msg  Request message.
 * @return   none
 */
void ui_handle_feature_req(struct cfw_message *msg)
{
	ui_service_get_available_features_rsp_t *resp;

	resp = (ui_service_get_available_features_rsp_t *)cfw_alloc_rsp_msg(
		msg, MSG_ID_UI_SERVICE_GET_AVAILABLE_FEATURES_RSP, sizeof(*resp));
	if (resp == NULL) panic(PANIC_FAIL_1);

	resp->status = 0;
	resp->events = ui_available_events;
	resp->notifications = ui_available_notifications;

	cfw_send_message(resp);
}

/*
 * Function to handle requests to get current enabled events.
 *
 * @details Reponse build with the current available and enabled events
 *          and sent back via the framework.
 * @param[in]  msg  Request message.
 * @return   none
 */
void ui_handle_get_enabled_event_req(struct cfw_message *msg)
{
	ui_service_get_events_rsp_t *resp;

	resp = (ui_service_get_events_rsp_t *)cfw_alloc_rsp_msg(
		msg, MSG_ID_UI_SERVICE_GET_ENABLED_EVENT_RSP, sizeof(*resp));
	if (resp == NULL) panic(PANIC_FAIL_1);

	resp->status = 0;
	resp->events = ui_available_events_enabled;

	cfw_send_message(resp);
}

/*
 * Function to return mask of current available events.
 *
 * @return   Event mask of available events.
 */
uint32_t ui_get_events_list(void)
{
	pr_debug(LOG_MODULE_UI_SVC,
		 "Available events: 0x%X",
		 ui_available_events);

	return ui_available_events;
}

/*
 * Function to initialize UI Service.
 *
 * @details Internal variable and structures are intialized, service and
 *          port handles allocated by the framework. UI service registers
 *          to the framework.
 * @param[in]  ui_svc_queue Queue of messages to be exchanged.
 * @return   none
 */
static void ui_service_init(int service_id, T_QUEUE ui_svc_queue)
{
	ui_available_events = 0x0;

	/* Initialize available events and notifications. */

	/* FIXME: May have to be modified when default configuration for device */
	/* platform will be defined. If a default configuration is coded in a   */
	/* dedicated place (settings ?) we may have to set ui_available_events, */
	/* ui_available_events_enabled, ui_available_notifications from it.     */
#ifdef CONFIG_UI_SERVICE_IMPL_BUTTON
	ui_available_events |= UI_BTN_SINGLE_EVT_IDX | UI_BTN_DOUBLE_EVT_IDX |
			       UI_BTN_MAX_EVT_IDX | UI_BTN_MULTIPLE_EVT_IDX;
#endif
#ifdef TEST_UI_SVC
	ui_available_events |= UI_TEST_EVT_IDX;
#endif

	ui_available_events_enabled = 0;
	ui_available_notifications = UI_LED_NOTIF_IDX;

	/* Init list of services that will be opened by the ui service when */
	/* requests for settings events.                                    */
	ui_init_client_service_table();

	if (cfw_register_service(ui_svc_queue,
				 &ui_service,
				 ui_handle_message,
				 NULL) == -1) {
		pr_error(LOG_MODULE_UI_SVC, "Cannot register UI service");
		return;
	}

	ui_svc_queue_msg = ui_svc_queue;

	/* Initialize interface with drivers. */
#ifdef CONFIG_UI_SERVICE_IMPL_BUTTON
	if (ui_config.btns) {
		button_init(ui_config.btns, ui_config.btn_count,
			    ui_button_notification_callback);
	}
#endif
#ifdef CONFIG_UI_SERVICE_IMPL_LED
	/* LED driver. */
	if (ui_config.led_count) {
		led_set_pattern_callback(ui_play_led_callback,
					 ui_config.led_count,
					 ui_config.leds);
	}
#endif
#ifdef CONFIG_UI_SERVICE_IMPL_HAPTIC
	/* Haptic driver. */
	extern struct device DEVICE_NAME_GET(haptic);
	haptic_dev = DEVICE_GET(haptic);
	struct haptic_config cfg;
	cfg.evt_callback_fn = ui_play_vibr_callback;

	haptic_set_config(haptic_dev, &cfg);
#endif

	/* UI service ready to accept new requests for setting events. */
	ui_svc_handler_state = UI_SET_EVT_READY;

	/* Semaphore to sync up on led play request during shutdown */
	led_play_done = semaphore_create(1);

	/* Semaphore to sync up on vibra play request during shutdown */
	vibr_play_done = semaphore_create(1);
}

CFW_DECLARE_SERVICE(UI, UI_SVC_SERVICE_ID, ui_service_init);

/*
 * Function to stop UI Service.
 *
 * @details The service unregisters from the framework.
 * @return   UI_SUCCESS on success, else error code.
 */
int8_t ui_service_stop(void)
{
	if (!cfw_unregister_service(&ui_service)) {
		pr_debug(LOG_MODULE_UI_SVC,
			 "Error: cannot unregister UI service");
		return UI_ERROR;
	}

	return UI_SUCCESS;
}
