/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/types.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/gfp.h>
#include <mach/msm_bus.h>

#include "vpu_bus_clock.h"
#include "vpu_debug.h"
#include "vpu_resources.h"

#ifndef VPU_MAPLE_FW_SIM

struct _vpu_bus_ctrl {
	u32 bus_client;
	struct bus_load_tbl *btabl;
};

static struct _vpu_bus_ctrl	g_vpu_bus_ctrl;

void vpu_bus_deinit(void)
{
	struct _vpu_bus_ctrl *ctrl;

	/* currently using static var, may change to dynamic allocation */
	ctrl = &g_vpu_bus_ctrl;

/*	if (!ctrl)
		return;
*/

	if (ctrl->bus_client) {
		msm_bus_scale_unregister_client(ctrl->bus_client);
		ctrl->bus_client = 0;
	}
}

int vpu_bus_init(struct vpu_platform_resources *res)
{
	struct _vpu_bus_ctrl *ctrl;
	int rc = 0;

	if (res->bus_table.count == 0)
		return -EINVAL;

	/* currently using static var, may change to dynamic allocation */
	ctrl = &g_vpu_bus_ctrl;
	ctrl->btabl = &res->bus_table;

	ctrl->bus_client =
		msm_bus_scale_register_client(&res->bus_pdata);
	if (!ctrl->bus_client) {
		dprintk(VPU_ERR, "Failed to register bus scale client\n");
		goto err_init_bus;
	}

	return rc;
err_init_bus:
	vpu_bus_deinit();
	return -EINVAL;
}

static int _get_bus_vector(struct _vpu_bus_ctrl *ctrl, int load)
{
	int i, j;
	int num_rows = ctrl->btabl ? ctrl->btabl->count : 0;

	if (num_rows <= 1)
		return 0;

	for (i = 0; i < num_rows; i++) {
		if (load <= ctrl->btabl->loads[i])
			break;
	}

	j = (i < num_rows) ? i : num_rows - 1;

	dprintk(VPU_DBG, "Required bus = %d\n", j);
	return j;
}

int vpu_bus_scale(int load)
{
	int rc = 0;
	u32 handle = 0;
	struct _vpu_bus_ctrl *ctrl = &g_vpu_bus_ctrl;

	if (!ctrl) {
		dprintk(VPU_ERR, "%s invalid ctrl handle %p\n",
			__func__, ctrl);
		return -EINVAL;
	}

	handle = ctrl->bus_client;

	if (handle) {
		rc = msm_bus_scale_client_update_request(
				handle, _get_bus_vector(ctrl, load));
		if (rc)
			dprintk(VPU_ERR, "Failed to scale bus: %d\n", rc);
	}

	return rc;
}

int vpu_bus_vote(void)
{
	int rc = 0;
	u32 handle = 0;
	struct _vpu_bus_ctrl *ctrl = &g_vpu_bus_ctrl;

	if (!ctrl) {
		dprintk(VPU_ERR, "%s invalid ctrl handle %p\n",
			__func__, ctrl);
		return -EINVAL;
	}

	handle = ctrl->bus_client;

	if (handle) {
		rc = msm_bus_scale_client_update_request(
				handle, 7);
		if (rc)
			dprintk(VPU_ERR, "Failed to vote bus: %d\n", rc);
	}

	return rc;
}

int vpu_bus_unvote(void)
{
	int rc = 0;
	u32 handle = 0;
	struct _vpu_bus_ctrl *ctrl = &g_vpu_bus_ctrl;

	if (!ctrl) {
		dprintk(VPU_ERR, "%s invalid ctrl handle %p\n",
			__func__, ctrl);
		return -EINVAL;
	}

	handle = ctrl->bus_client;

	if (handle) {
		rc = msm_bus_scale_client_update_request(
				handle, 0);
		if (rc)
			dprintk(VPU_ERR, "Failed to unvote bus: %d\n", rc);
	}

	return rc;
}

/*
 * Here’s the list of clks going into HQV:
 * clock name:			normal/turbo/SVS (MHz)
 * vpu_ahb_clk			80 /80 /40
 * vpu_axi_clk			333/466/150
 * vpu_bus_clk			80 /80 /40
 * vpu_maple_clk		400/400/200
 * vpu_vdp_clk			320/400/200
 * vpu_qdss_apb_clk
 * vpu_qdss_at_clk
 * vpu_qdss_tsctr_div8_clk
 * vpu_sleep_clk	// qtimer when xo is disabled, watchdog
 * vpu_cxo_clk		//qtimer in active mode

 * The mmcc_vpu_ahb_clk, mmcc_vpu_maple_axi_clk, and mmcc_vpu_axi_clk and the
 * will be subject to DCD frequency changes
 * There is a case where for power consumption we may witch to switch the
 * mmcc_vpu_vdp_clk between 200MHz and 400MHz during runtime to optimize for
 * power consumption
 */
#define	VPU_CLK_GATE_LEVEL VPU_VDP_CLK

const char *clock_names[VPU_MAX_CLKS] = {
	[VPU_BUS_CLK] = "vdp_bus_clk",
	[VPU_MAPLE_CLK] = "core_clk",
	[VPU_VDP_CLK] = "vdp_clk",
	[VPU_AHB_CLK] = "iface_clk",
	[VPU_AXI_CLK] = "bus_clk",
	[VPU_SLEEP_CLK] = "sleep_clk",
	[VPU_CXO_CLK] = "cxo_clk",
	[VPU_MAPLE_AXI_CLK] = "maple_bus_clk",
	[VPU_PRNG_CLK] = "prng_clk",
};

struct vpu_core_clock {
	struct clk *clk;
	u32 status;
	u32 current_freq;
	struct load_freq_table *load_freq_tbl;
	const char *name;
};

/*
 * Note: there is no lock in this block
 * It is caller's responsibility to serialize the calls
 */
struct vpu_clk_control {
	/* clock status */
	u32 load;

	struct vpu_core_clock clock[VPU_MAX_CLKS];
};

void *vpu_clock_init(struct vpu_platform_resources *res)
{
	int i;
	int rc = -1;
	struct vpu_core_clock *cl;
	struct vpu_clk_control *clk_ctrl;

	if (!res)
		return NULL;

	clk_ctrl = (struct vpu_clk_control *)
			kzalloc(sizeof(struct vpu_clk_control), GFP_KERNEL);
	if (!clk_ctrl) {
		dprintk(VPU_ERR, "failed to allocate clock ctrl block\n");
		return NULL;
	}

	/* setup the clock handles */
	for (i = 0; i < VPU_MAX_CLKS; i++) {
		cl = &clk_ctrl->clock[i];

		cl->load_freq_tbl = &res->clock_tables[i];
		cl->name = clock_names[i];

		if (i <= VPU_CLK_GATE_LEVEL && cl->load_freq_tbl->count == 0) {
			dprintk(VPU_ERR, "%s freq table size is 0\n", cl->name);
			goto fail_init_clocks;
		}

		cl->clk = devm_clk_get(&res->pdev->dev, cl->name);
		if (IS_ERR_OR_NULL(cl->clk)) {
			dprintk(VPU_ERR,
				"Failed to get clock: %s\n", cl->name);
			rc = PTR_ERR(cl->clk);
			break;
		}
		cl->status = 0;
	}

	/* free the clock if not all successful */
	if (i < VPU_MAX_CLKS) {
		for (i = 0; i < VPU_MAX_CLKS; i++) {
			cl = &clk_ctrl->clock[i];
			if (cl->clk) {
				clk_put(cl->clk);
				cl->clk = NULL;
			}
		}
		goto fail_init_clocks;
	}

	return clk_ctrl;

fail_init_clocks:
	kfree(clk_ctrl);
	return NULL;
}

void vpu_clock_deinit(void *clkh)
{
	int i;
	struct vpu_core_clock *cl;
	struct vpu_clk_control *clk_ctr = (struct vpu_clk_control *)clkh;

	if (!clk_ctr) {
		dprintk(VPU_ERR, "Invalid param\n");
		return;
	}

	for (i = 0; i < VPU_MAX_CLKS; i++) {
		cl = &clk_ctr->clock[i];
		if (cl->status) {
			clk_disable_unprepare(cl->clk);
			cl->status = 0;
		}
		clk_put(cl->clk);
		cl->clk = NULL;
	}

	kfree(clk_ctr);
}


void vpu_clock_disable(void *clkh)
{
	int i;
	struct vpu_core_clock *cl;
	struct vpu_clk_control *clk_ctr = (struct vpu_clk_control *)clkh;

	if (!clk_ctr) {
		dprintk(VPU_ERR, "Invalid param: %p\n", clk_ctr);
		return;
	}

	for (i = 0; i < VPU_MAX_CLKS; i++) {
		cl = &clk_ctr->clock[i];
		if (cl->status) {
			clk_disable_unprepare(cl->clk);
			cl->status = 0;
		}
	}
}

int vpu_clock_enable(void *clkh)
{
	struct vpu_core_clock *cl;
	struct vpu_clk_control *clk_ctr = (struct vpu_clk_control *)clkh;
	int i = 0;
	int rc = 0;
	VPU_ENTER_FUNC();

	if (!clk_ctr) {
		dprintk(VPU_ERR, "Invalid param: %p\n", clk_ctr);
		return -EINVAL;
	}

	for (i = 0; i < VPU_MAX_CLKS; i++) {
		cl = &clk_ctr->clock[i];

		if (cl->status == 0) {
			/* set rate if it's a gated clock */
			if (i <= VPU_CLK_GATE_LEVEL &&
					cl->load_freq_tbl->entry) {
				cl->current_freq =
					cl->load_freq_tbl->entry[0].freq;
				rc = clk_set_rate(cl->clk, cl->current_freq);
			}
			if (rc) {
				dprintk(VPU_ERR, "Failed to set rate for %s\n",
						cl->name);
				goto fail_clk_enable;
			}

			rc = clk_prepare_enable(cl->clk);
			if (rc) {
				dprintk(VPU_ERR,
					"Failed to enable clock %s (err %d)\n",
					cl->name, rc);
				goto fail_clk_enable;
			} else {
				dprintk(VPU_DBG, "%s prepare_enabled\n",
						cl->name);
				cl->status = 1;
			}
		}
	}

	return rc;

fail_clk_enable:
	for (i = 0; i < VPU_MAX_CLKS; i++) {
		cl = &clk_ctr->clock[i];
		if (cl->status) {
			clk_disable_unprepare(cl->clk);
			cl->status = 0;
		}
	}

	return rc;
}

void vpu_clock_gating_on(void *clkh)
{
	int i;
	struct vpu_core_clock *cl;
	struct vpu_clk_control *clk_ctr = (struct vpu_clk_control *)clkh;

	if (!clk_ctr) {
		dprintk(VPU_ERR, "Invalid param: %p\n", clk_ctr);
		return;
	}

	for (i = 0; i <= VPU_CLK_GATE_LEVEL; i++) {
		cl = &clk_ctr->clock[i];
		if (cl->status) {
			clk_disable(cl->clk);
			cl->status = 0;
		}
	}

	return;
}

int vpu_clock_gating_off(void *clkh)
{
	int i;
	struct vpu_core_clock *cl;
	struct vpu_clk_control *clk_ctr = (struct vpu_clk_control *)clkh;

	int rc = 0;
	if (!clk_ctr) {
		dprintk(VPU_ERR, "Invalid param: %p\n", clk_ctr);
		return -EINVAL;
	}

	for (i = 0; i <= VPU_CLK_GATE_LEVEL; i++) {
		cl = &clk_ctr->clock[i];
		if (cl->status == 0) {
			rc = clk_enable(cl->clk);
			if (rc) {
				dprintk(VPU_ERR, "%s, Failed to enable %s\n",
						__func__, cl->name);
				break;
			} else {
				cl->status = 1;
				dprintk(VPU_DBG, "%s enabled\n", cl->name);
			}
		}
	}

	return rc;
}

static unsigned long _clock_get_rate(struct vpu_core_clock *clock,
	u32 num_mbs_per_sec) /* FIXME: today, we receive load in bits/sec */
{
	struct load_freq_table *table = clock->load_freq_tbl;
	unsigned long ret = 0;
	int i;

	for (i = 0; i < table->count; i++) {
		ret = table->entry[i].freq;
		if (num_mbs_per_sec <= table->entry[i].load)
			break;
	}
	dprintk(VPU_INFO, "Required clock rate = %lu\n", ret);
	return ret;
}

int vpu_clock_scale(void *clkh, u32 load)
{
	struct vpu_clk_control *clk_ctr = (struct vpu_clk_control *)clkh;
	int i, rc = 0;

	if (!clk_ctr) {
		dprintk(VPU_ERR, "Invalid param: %p\n", clk_ctr);
		return -EINVAL;
	}

	clk_ctr->load = load;

	for (i = 0; i <= VPU_CLK_GATE_LEVEL; i++) {
		struct vpu_core_clock *cl = &clk_ctr->clock[i];
		unsigned long freq;

		freq = _clock_get_rate(cl, load);
		rc = clk_set_rate(cl->clk, freq);

		if (rc)
			dprintk(VPU_ERR, "clk_set_rate failed %s rate: %lu\n",
					cl->name, freq);
		else
			cl->current_freq = freq;
	}

	return rc;
}

#else /* VPU_MAPLE_FW_SIM */

void vpu_bus_deinit(void)
{
	return;
}

int vpu_bus_init(struct vpu_platform_resources *res)
{
	return 0;
}

int vpu_bus_scale(int load)
{
	return 0;
}

int vpu_bus_vote(void)
{
	return 0;
}

int vpu_bus_unvote(void)
{
	return 0;
}

void *vpu_clock_init(struct vpu_platform_resources *res)
{
	return (void *) 0xDEADBEEF;
}

void vpu_clock_deinit(void *clkh)
{
	return;
}


void vpu_clock_disable(void *clkh)
{
	return;
}

int vpu_clock_enable(void *clkh)
{
	return 0;
}

void vpu_clock_gating_on(void *clkh)
{
	return;
}

int vpu_clock_gating_off(void *clkh)
{
	return 0;
}

int vpu_clock_scale(void *clkh, int load)
{
	return 0;
}

#endif /* VPU_MAPLE_FW_SIM */
