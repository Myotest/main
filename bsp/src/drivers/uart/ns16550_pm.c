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

#include <stdint.h>
#include "machine.h"
#include "infra/device.h"
#include "infra/log.h"
#include "drivers/ns16550_pm.h"
#include <init.h>

#define DRV_NAME "uart_pm"
#define LSR_RXRDY 0x01 /* receiver data available */

/* FIX ME: remove when IRQs are called with private parameter */
static void ns16550_pm_uart_ISR(struct td_device *dev);
static struct td_device *ns16550_pm_isr;

void uart_rx_isr(void *unused)
{
	ns16550_pm_uart_ISR(ns16550_pm_isr);
}

static void uart_console_init(struct td_device *dev)
{
	struct ns16550_pm_device *pmdev = dev->priv;

	irq_connect_dynamic(pmdev->vector, ISR_DEFAULT_PRIO, uart_rx_isr, 0, 0);
	irq_enable(pmdev->vector);

	/* Enable interrupt */
	SOC_UNMASK_INTERRUPTS(pmdev->uart_int_mask);

	/* Enable IRQ at controller level */
	uart_irq_rx_enable(pmdev->zephyr_device);
	/* allow detecting uart break */
	uart_irq_err_enable(pmdev->zephyr_device);
}

static int ns16550_pm_init(struct td_device *td_dev)
{
	/* Re-init uart with device settings */
	ns16550_pm_isr = td_dev;

	/* Re-init hardware : assumed to be done by Zephyr before this point */
	//uart_init(dev);

	uart_console_init(td_dev);

	return 0;
}

static int ns16550_pm_suspend(struct td_device *td_dev, PM_POWERSTATE state)
{
	return 0;
}

static int ns16550_pm_resume(struct td_device *td_dev)
{
	struct ns16550_pm_device *pmdev = td_dev->priv;

	pmdev->zephyr_device->config->init(pmdev->zephyr_device);
	uart_console_init(td_dev);
	return 0;
}

void ns16550_pm_uart_ISR(struct td_device *dev)
{
	struct ns16550_pm_device *pmdev = (struct ns16550_pm_device *)dev->priv;

	if (pmdev->uart_rx_callback)
		pmdev->uart_rx_callback();
}

struct driver ns16550_pm_driver = {
	.init = ns16550_pm_init,
	.suspend = ns16550_pm_suspend,
	.resume = ns16550_pm_resume,
};
