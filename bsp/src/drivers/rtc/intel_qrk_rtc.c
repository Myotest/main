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

/*
 * Intel Quark Real Time Clock driver
 *
 */
#include <nanokernel.h>
#include <rtc.h>

#include "machine.h"
#include "infra/log.h"
#include "infra/pm.h"
#include "drivers/clk_system.h"
#include "drivers/data_type.h"
#include "drivers/ioapic.h"
#include "os/os.h"
#include "util/assert.h"

#define RTC_WAKELOCK_DELAY            4000       /**< (ms) period to call rtc timer callback*/
#define RTC_DRV_NAME "RTC"
static struct pm_wakelock rtc_wakelock;
static T_TIMER rtc_wakelock_timer = NULL;
static bool is_rtc_init_done = false;
static void (*callback_fn)(struct device *dev);
static void rtc_isr(void *arg);
int qrk_cxxxx_rtc_init(struct device *rtc_dev);

struct clk_gate_info_s rtc_clk_gate_info = {
	.clk_gate_register = PERIPH_CLK_GATE_CTRL,
	.bits_mask = RTC_CLK_GATE_MASK,
};

void rtc_resume(void)
{
	extern struct device DEVICE_NAME_GET(rtc);
	IRQ_CONNECT(SOC_RTC_INTERRUPT, ISR_DEFAULT_PRIO, rtc_isr,
		    DEVICE_GET(rtc), IOAPIC_LEVEL | IOAPIC_HIGH);
	irq_enable(SOC_RTC_INTERRUPT);
}

static void qrk_cxxxx_rtc_wakelock_timer_callback(void *data)
{
	pm_wakelock_release(&rtc_wakelock);
}

static void qrk_cxxxx_rtc_clock_frequency(uint32_t frequency)
{
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
			       SCSS_CCU_SYS_CLK_CTL_OFFSET) &=
		~SCSS_CCU_RTC_CLK_DIV_EN;
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
			       SCSS_CCU_SYS_CLK_CTL_OFFSET) &=
		~SCSS_RTC_CLK_DIV_MASK;
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
			       SCSS_CCU_SYS_CLK_CTL_OFFSET) |= frequency;
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
			       SCSS_CCU_SYS_CLK_CTL_OFFSET) |=
		SCSS_CCU_RTC_CLK_DIV_EN;
}

/*! \fn     void qrk_cxxxx_rtc_enable(void)
 *
 *  \brief   Function to enable clock gating for the RTC
 */
static void qrk_cxxxx_rtc_enable(struct device *rtc_dev)
{
	set_clock_gate(&rtc_clk_gate_info, CLK_GATE_ON);
}

/*! \fn     void rtc_isr(void)
 *
 *  \brief   RTC alarm ISR, if specified calls a user defined callback
 */
static void rtc_isr(void *arg)
{
	struct device *dev = arg;

	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR, QRK_RTC_EOI);

	if (callback_fn)
		(*callback_fn)(dev);
}

static void qrk_cxxxx_rtc_one_time_setup(struct device *rtc_dev)
{
	OS_ERR_TYPE err;
	const uint32_t expected_freq = SCSS_RTC_CLK_DIV_1_HZ |
				       SCSS_CCU_RTC_CLK_DIV_EN;

	qrk_cxxxx_rtc_enable(rtc_dev);
	rtc_wakelock_timer = timer_create(qrk_cxxxx_rtc_wakelock_timer_callback,
					  NULL,
					  RTC_WAKELOCK_DELAY,
					  false,
					  false,
					  &err);
	if (E_OS_OK != err)
		pr_error(LOG_MODULE_DRV, "rtc_wakelock_timer err");

	uint32_t curr_freq =
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
				       SCSS_CCU_SYS_CLK_CTL_OFFSET) &
		(SCSS_CCU_RTC_CLK_DIV_EN | SCSS_RTC_CLK_DIV_MASK);
	pm_wakelock_init(&rtc_wakelock);

	/* disable interrupt */
	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
			       QRK_RTC_CCR) &= ~QRK_RTC_INTERRUPT_ENABLE;
	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR, QRK_RTC_EOI);

	/* Reset initial value only if RTC wasn't enabled at right frequency at
	 * beginning of init
	 */
	if (expected_freq != curr_freq) {
		/* Set RTC divider 4096HZ for fast update */
		qrk_cxxxx_rtc_clock_frequency(SCSS_RTC_CLK_DIV_4096_HZ);

		/* set intial RTC value 0 */
		MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR, QRK_RTC_CLR) = 0;
		while (0 !=
		       MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
					      QRK_RTC_CCVR)) {
			MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
					       QRK_RTC_CLR) = 0;
		}
	}
}

/*! \fn     int qrk_cxxxx_rtc_set_config(struct device *dev, struct rtc_config *config)
 *
 *  \brief   Function to configure the RTC
 *
 *  \param   config   : pointer to a RTC configuration structure
 *
 *  \return  DEV_OK on success\n
 *           DEV_USED otherwise
 */
static int qrk_cxxxx_rtc_set_config(struct device *	rtc_dev,
				    struct rtc_config * config)
{
	OS_ERR_TYPE err;

	/*  Set RTC divider - 32.768khz / 32768 = 1 second.
	 *   Note: Divider not implemented in standard emulation image.
	 */
	if (!is_rtc_init_done) {
		qrk_cxxxx_rtc_one_time_setup(rtc_dev);
		is_rtc_init_done = true;
	}
	/* Set RTC divider 1HZ */
	qrk_cxxxx_rtc_clock_frequency(SCSS_RTC_CLK_DIV_1_HZ);
	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
			       QRK_RTC_CCR) |= QRK_RTC_INTERRUPT_MASK;

	if (false != config->alarm_enable) {
		if (config->cb_fn)
			callback_fn = config->cb_fn;
	} else {
		/* set initial RTC value */
		while (config->init_val !=
		       MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
					      QRK_RTC_CCVR)) {
			MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
					       QRK_RTC_CLR) = config->init_val;
		}
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
				       SCSS_INT_RTC_MASK_OFFSET) = ~(0);
	}
	pm_wakelock_acquire(&rtc_wakelock);
	timer_start(rtc_wakelock_timer, RTC_WAKELOCK_DELAY, &err);
	if (err != E_OS_OK) {
		pm_wakelock_release(&rtc_wakelock);
		return DEV_USED;
	}
	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
			       QRK_RTC_CCR) &= ~QRK_RTC_INTERRUPT_MASK;
	return DEV_OK;
}

/*! \fn     uint32_t qrk_cxxxx_rtc_read(void)
 *
 *  \brief   Function to read the RTC
 *
 *  \return  uint32_t - epoch time
 */
static uint32_t qrk_cxxxx_rtc_read(struct device *rtc_dev)
{
	return MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR, QRK_RTC_CCVR);
}

/*! \fn     void qrk_cxxxx_rtc_disable(void)
 *
 *  \brief   Function to enable clock gating for the RTC
 */
void qrk_cxxxx_rtc_disable(struct device *rtc_dev)
{
	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
			       QRK_RTC_CCR) &= ~QRK_RTC_ENABLE;
	set_clock_gate(&rtc_clk_gate_info, CLK_GATE_OFF);
}

/*! \fn     int qrk_cxxxx_rtc_set_alarm(struct device *rtc_dev, const uint32_t alarm_val)
 *
 *  \brief  Function to set an RTC alarm
 *
 *  \param  alarm_val Alarm value
 *
 *  \return DEV_OK on success
 *          DEV_USED otherwise
 */
int qrk_cxxxx_rtc_set_alarm(struct device *rtc_dev, const uint32_t alarm_val)
{
	OS_ERR_TYPE err;

	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
			       QRK_RTC_CCR) &= ~QRK_RTC_INTERRUPT_ENABLE;

	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR, QRK_RTC_EOI);
	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR, QRK_RTC_CMR) = alarm_val;

	irq_enable(SOC_RTC_INTERRUPT);

	/* unmask RTC interrupts to quark  */
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_INT_RTC_MASK_OFFSET) =
		QRK_INT_RTC_UNMASK_QRK;

	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
			       QRK_RTC_CCR) |= QRK_RTC_INTERRUPT_ENABLE;
	MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR,
			       QRK_RTC_CCR) &= ~QRK_RTC_INTERRUPT_MASK;

	pm_wakelock_acquire(&rtc_wakelock);
	timer_start(rtc_wakelock_timer, RTC_WAKELOCK_DELAY, &err);
	if (err != E_OS_OK) {
		pm_wakelock_release(&rtc_wakelock);
		return DEV_USED;
	}
	return DEV_OK;
}

static struct rtc_driver_api qrk_cxxxx_rtc_funcs = {
	.set_config = qrk_cxxxx_rtc_set_config,
	.read = qrk_cxxxx_rtc_read,
	.enable = qrk_cxxxx_rtc_enable,
	.disable = qrk_cxxxx_rtc_disable,
	.set_alarm = qrk_cxxxx_rtc_set_alarm,
};

DEVICE_INIT(rtc, RTC_DRV_NAME, &qrk_cxxxx_rtc_init,
	    NULL, NULL,
	    SECONDARY, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

int qrk_cxxxx_rtc_init(struct device *rtc_dev)
{
	IRQ_CONNECT(SOC_RTC_INTERRUPT, ISR_DEFAULT_PRIO, rtc_isr,
		    DEVICE_GET(rtc), IOAPIC_LEVEL | IOAPIC_HIGH);
	irq_enable(SOC_RTC_INTERRUPT);
	rtc_dev->driver_api = &qrk_cxxxx_rtc_funcs;
	return 0;
}
