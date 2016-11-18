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

#include <zephyr.h>
#include <errno.h>

#include "machine/soc/intel/quark_se/pm_pupdr.h"
#include "infra/device.h"
#include "infra/soc_reboot.h"
#include "infra/ipc.h"
#include "infra/panic.h"
#include "infra/log.h"
#include "infra/time.h"
#include "infra/pm.h"

#include "deep_sleep.h"
#include "drivers/usb_pm.h"
#include "drivers/clk_system.h"

#include "machine.h"
#include "machine/soc/intel/quark_se/quark/reboot_reg.h"
#include "machine/soc/intel/quark_se/quark/soc_setup.h"
#include "quark_se_common.h"
#include "soc_config.h"

#define PANIC_SHUTDOWN_FAILED           1       /*!< Panic error code for shutdown failures   */
#define PANIC_ARC_SUSPEND_TIMEOUT       0x25    /*!< Panic error code for arc suspend timeout */
#define PANIC_ARC_SHUTDOWN_TIMEOUT      0x26    /*!< Panic error code for arc suspend timeout */
#define PANIC_ARC_RESUME_TIMEOUT        0x27    /*!< Panic error code for arc resume timeout  */
#define PANIC_ARC_RESUME_ERROR          0x28    /*!< Panic error code for arc resume error    */

#define SEC_TO_EPOCH 1

#define UPDATE_DELAY 5

/*!< wait 10 s timeout for ARC suspend and wakelocks expiration (before PANIC) */
#define IPC_PM_SUSPEND_TIMEOUT (10000)

int pm_wait_suspend_request_ack()
{
	uint32_t start_time = get_uptime_ms();

	while ((get_uptime_ms() - start_time) < IPC_PM_SUSPEND_TIMEOUT) {
		// Check for ack notification from ARC
		if (PM_IS_ACK_OK(shared_data->pm_request)) return PM_ACK_OK;
		else if (PM_IS_ACK_ERROR(shared_data->pm_request)) return
				PM_ACK_ERROR;
		// Abort if slave core is stopped or waiting for ipc acknowledge
		if (MBX_STS(IPC_SS_QRK_REQ) || !shared_data->arc_ready) {
			return PM_ACK_ERROR;
		}
	}
	// Timeout occured
	return PM_ACK_TIMEOUT;
}

#ifdef CONFIG_DEEPSLEEP

static volatile int arc_ack_received = 0;
static volatile int arc_status = 0;
int pm_core_deepsleep()
{
	int ret = -1;
	int ack;
	volatile uint32_t *slp_cfg = &SCSS_REG_VAL(SLP_CFG_BASE);
	const uint32_t iostate_ret_mask =
		(SLP_CFG_VRET_SEL_135 | SLP_CFG_IO_STATE_RET_EN);

	// Sent PM request to ARC core
	PM_INIT_REQUEST(shared_data->pm_request, PM_SUSPEND_REQUEST,
			PM_SUSPENDED);
	// Wakeup ARC core so It can acknowledge the request quickly
	MBX_CTRL(IPC_QRK_SS_ASYNC) = 0x80000000;
	// Wait for ARC acknowledge
	ack = pm_wait_suspend_request_ack();
	if (ack == PM_ACK_TIMEOUT) panic(PANIC_ARC_SUSPEND_TIMEOUT);
	if (ack != PM_ACK_OK) {
		PM_INIT_REQUEST(shared_data->pm_request, 0, 0);
		goto failed;
	}

	/* Try to suspend devices before waiting for ARC to sleep.
	 * This avoids busy-looping too much in the next loop.*/
	ret = suspend_devices(PM_SUSPENDED);

	/* ensure arc is halted */
	uint32_t start_time = get_uptime_ms();
	while (!(SCSS_REG_VAL(SCSS_SS_STS) & SCSS_SS_STS_ARC_HALT)) {
		if ((get_uptime_ms() - start_time) > IPC_PM_SUSPEND_TIMEOUT) {
			// ARC halt timeout detected
			panic(PANIC_ARC_SUSPEND_TIMEOUT);
		}
	}

	if (ret != 0) {
		pr_warning(LOG_MODULE_QUARK_SE, "QRK suspend failed (%d)", ret);
		goto resume_arc;
	}

	/* Configure IO State Retention to hold output states on pins */
	*slp_cfg = iostate_ret_mask;
	/* Loop until the bit is set */
	while (!(*slp_cfg & iostate_ret_mask)) ;

	// Platform Saved ctx
	soc_suspend();
	enum oscillator osc = clk_get_oscillator();
	if (osc == CLK_OSC_EXTERNAL) {
		clk_set_oscillator(CLK_OSC_INTERNAL);
	}
	// Goto deep sleep mode
	quark_deep_sleep();
	if (osc == CLK_OSC_EXTERNAL) {
		clk_set_oscillator(CLK_OSC_EXTERNAL);
	}
	// Platform restore
	soc_resume();
	resume_devices();

resume_arc:
	// Init resume request
	PM_INIT_REQUEST(shared_data->pm_request, PM_RESUME_REQUEST, PM_RUNNING);

	/* Put ARC out of reset */
	SCSS_REG_VAL(SCSS_SS_CFG) = ARC_RUN_REQ_A;

	ack = pm_wait_suspend_request_ack();
	if (ack == PM_ACK_TIMEOUT) panic(PANIC_ARC_RESUME_TIMEOUT);
	if (ack != PM_ACK_OK) panic(PANIC_ARC_RESUME_ERROR);

	/* Release IO State Retention to re-connect output pins to SoC */
	*slp_cfg = SLP_CFG_IO_STATE_RET_HOLD | SLP_CFG_VRET_SEL_135;
failed:
	return ret;
}
#endif

static void set_boot_target(enum boot_targets boot_target)
{
	SET_REBOOT_REG(SCSS_GPS0, BOOT_TARGETS_MASK, BOOT_TARGETS_POS,
		       boot_target);
}

void power_off(enum shutdown_reason reason)
{
	pm_shutdown_request(PUPDR_POWER_OFF, reason);
}

void shutdown(enum shutdown_reason reason)
{
	pm_shutdown_request(PUPDR_SHUTDOWN, reason);
}

void emergency_shutdown(enum shutdown_reason reason)
{
	pm_shutdown_request(PUPDR_SHUTDOWN, reason);
	/* Request immediate reboot */
	irq_lock();
	pm_shutdown();
	/* We should not reach this line */
}

void reboot(enum boot_targets boot_target)
{
	pm_shutdown_request(PUPDR_REBOOT, boot_target);
}

__weak void board_poweroff_hook()
{
}

__weak void board_shutdown_hook()
{
}

void pm_core_shutdown(enum pupdr_request req, int param)
{
	if (req == PUPDR_POWER_OFF) {
		board_poweroff_hook();
	}
	if (req == PUPDR_SHUTDOWN) {
		board_shutdown_hook();
	}
	if (req == PUPDR_REBOOT) {
		/* Set the next boot target to let the bootloader know */
		set_boot_target(param);
		soc_reboot();
	} else {
		soc_shutdown();
	}
}

#ifdef CONFIG_DEEPSLEEP
__weak bool pm_is_core_deepsleep_allowed()
{
	if (pm_wakelock_are_other_cpu_wakelocks_taken()) {
		/* Wakelock on slave core */
		return false;
	}
	return true;
}
#endif

__weak bool pm_is_core_shutdown_allowed()
{
	return !shared_data->arc_ready;
}
