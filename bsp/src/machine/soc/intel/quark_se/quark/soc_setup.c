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

#include <misc/printk.h>
#include <microkernel.h>
#include <arch/cpu.h>
#include <limits.h>
#include <string.h>

/* Operating System Abstraction */
#include "os/os.h"
#include "util/assert.h"

/* Board/SOC */
#include "machine.h"
#include <uart.h>
#include <init.h>
#include "machine/soc/intel/quark_se/quark/soc_setup.h"
#include "infra/pm.h"
#include "infra/device.h"
#include "infra/port.h"
#include "machine/soc/intel/quark_se/usb_setup.h"
#include "pinmux.h"
#include "drivers/ioapic.h"
#include "drivers/loapic.h"
#include "drivers/system_timer.h"
#include "project_mapping.h"
#include "machine/soc/intel/quark_se/quark/uart_tcmd_client.h"
#include "machine/soc/intel/quark_se/quark/reboot_reg.h"
#include "infra/boot.h"
#include "infra/log.h"
#include "infra/ipc.h"
#include "util/compiler.h"
#include "pm_pinmux.h"
#include <board.h>
#include <device.h>
#include "quark_se_common.h"

#ifdef CONFIG_QUARK_SE_PANIC_DEFAULT
void _divTrapStub(void);
void _nmiTrapStub(void);
void _overflowTrapStub(void);
void _boundTrapStub(void);
void _invOptTrapStub(void);
void _devNotAvailTrapStub(void);
void _dblFaultTrapStub(void);
void _tssTrapStub(void);
void _segTrapStub(void);
void _stackTrapStub(void);
void _gpfTrapStub(void);
void _pageTrapStub(void);
void _floatTrapStub(void);
void _alignTrapStub(void);

NANO_CPU_INT_REGISTER(_divTrapStub, -1, -1, 0, 0);
NANO_CPU_INT_REGISTER(_nmiTrapStub, -1, -1, 2, 0);
NANO_CPU_INT_REGISTER(_overflowTrapStub, -1, -1, 4, 0);
NANO_CPU_INT_REGISTER(_boundTrapStub, -1, -1, 5, 0);
NANO_CPU_INT_REGISTER(_invOptTrapStub, -1, -1, 6, 0);
NANO_CPU_INT_REGISTER(_devNotAvailTrapStub, -1, -1, 7, 0);
NANO_CPU_INT_REGISTER(_dblFaultTrapStub, -1, -1, 8, 0);
NANO_CPU_INT_REGISTER(_tssTrapStub, -1, -1, 10, 0);
NANO_CPU_INT_REGISTER(_segTrapStub, -1, -1, 11, 0);
NANO_CPU_INT_REGISTER(_stackTrapStub, -1, -1, 12, 0);
NANO_CPU_INT_REGISTER(_gpfTrapStub, -1, -1, 13, 0);
NANO_CPU_INT_REGISTER(_pageTrapStub, -1, -1, 14, 0);
NANO_CPU_INT_REGISTER(_floatTrapStub, -1, -1, 16, 0);
NANO_CPU_INT_REGISTER(_alignTrapStub, -1, -1, 17, 0);

#ifdef CONFIG_PANIC_ON_BUS_ERROR
void _pmuIntStub(void);
#endif /* CONFIG_PANIC_ON_BUS_ERROR */

#endif

void mbx_out_channel()
{
	if (MBX_DAT1(4) == 0) {
		//pr_info(LOG_MODULE_QUARK_SE, "A%s", MBX_DAT0(4));
	} else {
		/* The logs use the default UART */
		extern struct device DEVICE_NAME_GET(uart_ns16550_1);
		uart_poll_out(DEVICE_GET(uart_ns16550_1), MBX_DAT0(4));
	}
}

/* TODO: refactor these 2 tables to a struct */
enum {
	IPC_RX_REQ = 0,
	IPC_RX_ACK,
	IPC_RX_ASYNC
};

static void (*ipc_callbacks[])() = {
	[IPC_RX_ASYNC] = mbx_out_channel,
	[IPC_RX_REQ] = NULL,
	[IPC_RX_ACK] = NULL
};

static const uint8_t ipc_channels[] = {
	[IPC_RX_ASYNC] = IPC_SS_QRK_ASYNC,
	[IPC_RX_REQ] = IPC_SS_QRK_REQ,
	[IPC_RX_ACK] = IPC_SS_QRK_ACK
};

void notrace mbxIsr(int param)
{
	do {
		int i;
		for (i = 0;
		     i < sizeof(ipc_channels) / sizeof(*ipc_channels);
		     i++) {
			int sts;
			sts = MBX_STS(ipc_channels[i]);
			if (sts & 0x2 && ipc_callbacks[i]) {
				/* Process ipc event */
				ipc_callbacks[i]();
			}
			/* Acknowledge ipc interrupt */
			if (ipc_channels[i] == IPC_SS_QRK_ASYNC)
				MBX_STS(ipc_channels[i]) = sts;
		}
	} while (MBX_CHALL_STS & 0x0500);
}

static void display_boot_target(void)
{
#ifndef NDEBUG
	enum boot_targets target = get_boot_target();
	printk("BOOT TARGET: %d\r\n", target);
#endif
}

#include <drivers/usb_api.h>
void soc_setup(void)
{
#ifdef CONFIG_USB
	IRQ_CONNECT(SOC_USB_INTERRUPT, ISR_DEFAULT_PRIO, usb_ISR, NULL, 0);
	irq_enable(SOC_USB_INTERRUPT);
#endif

	/* Temporary: we need to reference the IPC message handler from mbxIsr */
	ipc_callbacks[IPC_RX_REQ] = ipc_handle_message;

	/* TODO: Get the ARC Flash start address from predefined HW block */

	unsigned int *src = (unsigned int *)(ARC_FLASH_START_ADDR);
	uint32_t arc_state;
	int i;

	/* Copy arc vector to start of RAM. */
	shared_data->arc_start = *src;
	shared_data->arc_ready = 0;

	/* Check the revision of the SoC */
	/* Rev 1 silicon has bit 2 of SLP_CFG register tied to 1 */
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SLP_CFG_BASE) &= ~(0x4);

	/* Reset the Host voltage regulator ROKM bit */
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
			       HOST_VR_BASE) &= ~(HOST_VR_ROKM);

	printk("\n\n\n\nBOOT\n\n\nQuark SE ID %d Rev %d A%d\r\n",
	       SCSS_REG_VAL(SCSS_ID), SCSS_REG_VAL(SCSS_REV),
	       (MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
				       SLP_CFG_BASE) & 0x4) ? 1 : 0);

	/* Check Sensor Subsystem (ARC) status */
	arc_state = SCSS_REG_VAL(SCSS_SS_STS);
	if (arc_state == 0) {
		printk("ARC running on boot, halt !\r\n");
		SCSS_REG_VAL(SCSS_SS_CFG) |= ARC_HALT_REQ_A;
		disable_scheduling();
		irq_lock();
		while (1)
			mbxIsr(0);
	}
	printk("ARC Core state: %p\r\n", arc_state);

	/* Print Boot Target */
	display_boot_target();

	/* Clear Mailboxes */
	for (i = 0; i < 8; i++)
		MBX_STS(i) = 3;

	IRQ_CONNECT(SOC_MBOX_INTERRUPT, ISR_DEFAULT_PRIO, mbxIsr, NULL, 0);
	irq_enable(SOC_MBOX_INTERRUPT);

	SCSS_REG_VAL(SCSS_INT_MAILBOX_MASK) = 0xffffffff;
	/* Enable interrupt for ipc channels */
	SOC_MBX_INT_UNMASK(IPC_SS_QRK_ASYNC);
	SOC_MBX_INT_UNMASK(IPC_SS_QRK_REQ);

	/* Enable Always on timer */
	SCSS_REG_VAL(SCSS_AONC_CFG) = AONC_CNT_EN;

#ifdef CONFIG_QUARK_SE_DISABLE_INTERNAL_VREG
	/* disable platform voltage regulator */
	SCSS_REG_VAL(SCSS_PLAT3P3_VR) &= ~(PLAT_VR_ENABLE | PLAT_VR_HIZ);
	SCSS_REG_VAL(SCSS_PLAT1P8_VR) &= ~(PLAT_VR_ENABLE | PLAT_VR_HIZ);
#endif

#ifdef CONFIG_PANIC_ON_BUS_ERROR
	/* Clear bus error interrupt before activating */
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, QRK_SCSS_P_STS) = 8;
	/* Enable PMU interrupt */
	IRQ_CONNECT(SOC_SYSTEM_PMU_INTERRUPT, ISR_DEFAULT_PRIO, _pmuIntStub,
		    NULL,
		    0);
	irq_enable(SOC_SYSTEM_PMU_INTERRUPT);
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, INT_SYSTEM_PMU_MASK) &= ~(1);
#endif

	irq_enable(SOC_CMP_INTERRUPT);

	/* Init logs */
	log_init();

	/* init shared wakelock status variables */
	shared_data->any_lmt_wakelock_taken = 0;
	shared_data->any_arc_wakelock_taken = 0;

	/* Init wakelocks */
	pm_wakelock_init_mgr();

	/* Init devices */
	init_all_devices();

#ifdef CONFIG_USB
	/* Init SoC specific USB */
	platform_usb_init();
#endif

	local_task_sleep_ms(10);
}

void start_arc(unsigned int reset_vector)
{
	if (reset_vector != 0) {
		shared_data->arc_start = reset_vector;
	}

	shared_data->arc_ready = 0;
	shared_data->arc_next_wakeup_valid = false;
	shared_data->arc_next_wakeup = 0;
	shared_data->ports = port_get_port_table();
	shared_data->quark_cfw_ready = 0;

	/* Put ARC out of reset. */
	pr_debug(LOG_MODULE_QUARK_SE, "Put ARC out of reset");

	local_task_sleep_ms(10);

	/*
	 * Next line starts ARC execution.
	 */
	pr_debug(LOG_MODULE_QUARK_SE, "Start arc now");
	SCSS_REG_VAL(SCSS_SS_CFG) |= ARC_RUN_REQ_A;
	local_task_sleep_ms(10);

	pr_debug(LOG_MODULE_QUARK_SE, "ARC Core state: %p",
		 SCSS_REG_VAL(SCSS_SS_STS));

	while (!shared_data->arc_ready) ;
	pr_debug(LOG_MODULE_QUARK_SE, "ARC ready");
}

void soc_resume(void)
{
	// Resume devices managed by Zephyr OS

	pinmux_resume();

	/* LOAPIC initialisation*/
	extern int _loapic_init(struct device *unused);
	_loapic_init(NULL);
	/* IOAPIC initialisation*/
	extern int _ioapic_init(struct device *unused);
	_ioapic_init(NULL);
	/* System clock */
	_sys_clock_driver_init(NULL);

	/* Re-connect static irqs */
	extern void irq_resume();
	irq_resume();
	extern void rtc_resume(void);
	rtc_resume();
	irq_enable(SOC_CMP_INTERRUPT);
	irq_enable(SOC_MBOX_INTERRUPT);
}

void soc_suspend(void)
{
	pinmux_suspend();
}

bool pm_wakelock_are_other_cpu_wakelocks_taken(void)
{
	return shared_data->any_arc_wakelock_taken;
}

void notrace pm_wakelock_set_any_wakelock_taken_on_cpu(bool wl_status)
{
	shared_data->any_lmt_wakelock_taken = wl_status;
}
