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

#ifndef _H_VPU_RESOURCES_H_
#define _H_VPU_RESOURCES_H_

#include <linux/platform_device.h>
#include <mach/iommu_domains.h>
#include <mach/msm_bus.h>

/*****************************
 *** Device Tree Resources ***
 *****************************/

enum vpu_clocks {
	VPU_BUS_CLK,
	VPU_MAPLE_CLK,
	VPU_VDP_CLK,
	VPU_AHB_CLK,
	VPU_AXI_CLK,
	VPU_SLEEP_CLK,
	VPU_CXO_CLK,
	VPU_MAPLE_AXI_CLK,
	VPU_PRNG_CLK,
	VPU_MAX_CLKS
};

struct load_freq_pair {
	u32 load;
	u32 freq;
};

struct load_freq_table {
	struct load_freq_pair *entry;
	int count;
};

struct bus_load_tbl {
	u32 *loads;
	int count;
};

struct reg_value_pair {
	u32 reg;
	u32 value;
};

struct vpu_iommu_map {
	const char *client_name;
	const char *ctx_name;
	struct device *ctx;
	struct iommu_domain *domain;
	int domain_num;
	struct msm_iova_partition partitions[1];
	int npartitions;
	bool is_secure;
	bool enabled;
	bool attached;
};

struct iommu_set {
	struct vpu_iommu_map *iommu_maps;
	int count;
};

struct vpu_platform_resources {
	/* device register and mem window */
	phys_addr_t register_base_phy;
	phys_addr_t mem_base_phy;
	u32 register_size;
	u32 mem_size;

	/* interrupt number */
	u32 irq; /* Maple -> APPS IPC irq */
	u32 irq_wd; /* Maple's watchdog irq */

	struct load_freq_table clock_tables[VPU_MAX_CLKS];
	struct bus_load_tbl bus_table;
	struct msm_bus_scale_pdata bus_pdata;
	struct iommu_set iommu_set;

	struct platform_device *pdev;

	/* iommu memory device client */
	void *iommu_client;
};

int read_vpu_platform_resources(struct vpu_platform_resources *res,
			struct platform_device *pdev);
void free_vpu_platform_resources(struct vpu_platform_resources *res);

/*
 * IOMMU memory API
 */

/* Registration of IOMMU hardware domains */
int register_vpu_iommu_domains(struct vpu_platform_resources *res);
void unregister_vpu_iommu_domains(struct vpu_platform_resources *res);

/* Activation of IOMMU hardware domains */
int attach_vpu_iommus(struct vpu_platform_resources *res);
void detach_vpu_iommus(struct vpu_platform_resources *res);


/* Create/Destroy memory buffer allocation handle */
void *vpu_iommu_create_handle(struct vpu_platform_resources *res);
void vpu_iommu_destroy_handle(void *iommu_handle);

/* Map user allocated buffer memory */
int vpu_iommu_map_user_mem(void *iommu_handle, unsigned long fd,
		unsigned long length, unsigned long offset, bool secure);

/* Allocate in driver then map buffer memory */
int vpu_iommu_alloc_kernel_mem(void *iommu_handle, unsigned long size,
		bool secure);

/* Get iommu mapped address for buffer handle */
u32 vpu_iommu_get_addr(void *iommu_handle);

/* Get buffer size for corresponding handle */
u32 vpu_iommu_get_size(void *iommu_handle);

#endif /* _H_VPU_RESOURCES_H_ */
