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

#include <nanokernel.h>
#include "drivers/clk_system.h"
#include "infra/device.h"
#include "machine/soc/intel/quark_se/scss_registers.h"

void set_clock_gate(struct clk_gate_info_s *clk_gate_info, uint32_t value)
{
	uint32_t tmp;
	int flags = irq_lock();

	tmp = MMIO_REG_VAL(clk_gate_info->clk_gate_register);
	tmp &= ~(clk_gate_info->bits_mask);
	tmp |= ((clk_gate_info->bits_mask) & value);
	MMIO_REG_VAL(clk_gate_info->clk_gate_register) = tmp;

	irq_unlock(flags);
}

static int clk_system_driver_init(struct td_device *dev)
{
	set_clock_gate(dev->priv, CLK_GATE_OFF);
	return 0;
}

static int clk_system_driver_resume(struct td_device *dev)
{
	struct clk_gate_info_s *clk_gate_info =
		(struct clk_gate_info_s *)dev->priv;

	MMIO_REG_VAL(clk_gate_info->clk_gate_register) =
		clk_gate_info->bits_mask;
	return 0;
}

static int clk_system_driver_suspend(struct td_device *dev, PM_POWERSTATE state)
{
	struct clk_gate_info_s *clk_gate_info =
		(struct clk_gate_info_s *)dev->priv;

	/* Since bits_mask is only used for initialization, it can now be used to
	 * store the last register value.
	 */
	clk_gate_info->bits_mask = MMIO_REG_VAL(
		clk_gate_info->clk_gate_register);
	return 0;
}

enum oscillator clk_get_oscillator(void)
{
	uint32_t osc_cfg =
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1);

	return (osc_cfg & OSC0_CFG1_XTAL_OSC_EN_MASK) ?
	       CLK_OSC_EXTERNAL : CLK_OSC_INTERNAL;
}

#define INTERNAL_OSC_TRIM 0x240

void clk_set_oscillator(enum oscillator oscillator)
{
	if (oscillator == CLK_OSC_INTERNAL) {
		/* Start internal oscillator (with trim) */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) |=
			INTERNAL_OSC_TRIM << OSC0_CFG1_INTERNAL_OSC_TRIM_BIT |
			OSC0_CFG1_INTERNAL_OSC_EN_MASK;
		/* Wait internal oscillator ready */
		while (!((MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
						 SCSS_OSC0_STAT1)
			  & OSC0_STAT1_LOCK_INTERNAL))) ;
		/* Trim internal oscillator */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) =
			INTERNAL_OSC_TRIM << OSC0_CFG1_INTERNAL_OSC_TRIM_BIT |
			OSC0_CFG1_INTERNAL_OSC_EN_MASK;
	} else {
		/* Set clk to 32MHz, external oscillator, 5.5pF load */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) |=
			OSC0_CFG1_XTAL_OSC_TRIM_5_55_PF |
			OSC0_CFG1_XTAL_OSC_EN_MASK;
		/* Wait internal regulator ready */
		while (!((MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,
						 SCSS_OSC0_STAT1)
			  & OSC0_STAT1_LOCK_XTAL))) ;
		/* Switch to external oscillator */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) =
			OSC0_CFG1_XTAL_OSC_TRIM_5_55_PF |
			(OSC0_CFG1_XTAL_OSC_EN_MASK |
			 OSC0_CFG1_XTAL_OSC_OUT_MASK);
	}
}

struct driver clk_system_driver = {
	.init = clk_system_driver_init,
	.suspend = clk_system_driver_suspend,
	.resume = clk_system_driver_resume,
};
