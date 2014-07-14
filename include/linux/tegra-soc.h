/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_TEGRA_SOC_H_
#define __LINUX_TEGRA_SOC_H_

#define TEGRA20		0x20
#define TEGRA30		0x30
#define TEGRA114	0x35
#define TEGRA124	0x40

#define TEGRA_FUSE_SKU_CALIB_0	0xf0
#define TEGRA30_FUSE_SATA_CALIB	0x124

#ifndef __ASSEMBLY__

#include <linux/reboot.h>

u32 tegra_read_chipid(void);
u8 tegra_get_chip_id(void);

enum tegra_revision {
	TEGRA_REVISION_UNKNOWN = 0,
	TEGRA_REVISION_A01,
	TEGRA_REVISION_A02,
	TEGRA_REVISION_A03,
	TEGRA_REVISION_A03p,
	TEGRA_REVISION_A04,
	TEGRA_REVISION_MAX,
};

struct tegra_sku_info {
	int sku_id;
	int cpu_process_id;
	int cpu_speedo_id;
	int cpu_speedo_value;
	int cpu_iddq_value;
	int core_process_id;
	int soc_speedo_id;
	int gpu_speedo_id;
	int gpu_process_id;
	int gpu_speedo_value;
	enum tegra_revision revision;
};

u32 tegra_read_straps(void);
u32 tegra_read_chipid(void);
int tegra_fuse_readl(unsigned long offset, u32 *value);

extern struct tegra_sku_info tegra_sku_info;

/*
 * PMC
 */
enum tegra_suspend_mode {
	TEGRA_SUSPEND_NONE = 0,
	TEGRA_SUSPEND_LP2, /* CPU voltage off */
	TEGRA_SUSPEND_LP1, /* CPU voltage off, DRAM self-refresh */
	TEGRA_SUSPEND_LP0, /* CPU + core voltage off, DRAM self-refresh */
	TEGRA_MAX_SUSPEND_MODE,
};

#ifdef CONFIG_PM_SLEEP
enum tegra_suspend_mode tegra_pmc_get_suspend_mode(void);
void tegra_pmc_set_suspend_mode(enum tegra_suspend_mode mode);
void tegra_pmc_enter_suspend_mode(enum tegra_suspend_mode mode);

bool tegra_pmc_cpu_is_powered(int cpuid);
int tegra_pmc_cpu_power_on(int cpuid);
int tegra_pmc_cpu_remove_clamping(int cpuid);

void tegra_pmc_restart(enum reboot_mode mode, const char *cmd);
#endif

/*
 * PM
 */
#ifdef CONFIG_PM_SLEEP
enum tegra_suspend_mode
tegra_pm_validate_suspend_mode(enum tegra_suspend_mode mode);

/* low-level resume entry point */
void tegra_resume(void);
#else
static inline enum tegra_suspend_mode
tegra_pm_validate_suspend_mode(enum tegra_suspend_mode mode)
{
	return TEGRA_SUSPEND_NONE;
}

static inline void tegra_resume(void)
{
}
#endif /* CONFIG_PM_SLEEP */

#endif /* __ASSEMBLY__ */

#endif /* __LINUX_TEGRA_SOC_H_ */
