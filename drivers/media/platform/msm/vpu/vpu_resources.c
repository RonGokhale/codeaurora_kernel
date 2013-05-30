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

#include <linux/msm_ion.h>
#include <linux/iommu.h>
#include <mach/iommu.h>
#include <mach/iommu_domains.h>
#include <mach/msm_bus_board.h>
#include <asm-generic/sizes.h>

#include "vpu_resources.h"
#include "vpu_v4l2.h"
#include "vpu_debug.h"

/*
 * Device Tree Resources
 */

enum vpu_iommu_domains {
	VPU_HLOS_IOMMU_DOMAIN,
	VPU_CP_IOMMU_DOMAIN,
	VPU_FW_IOMMU_DOMAIN,
	VPU_MAX_IOMMU_DOMAIN,
};

static struct vpu_iommu_map iommus_array[VPU_MAX_IOMMU_DOMAIN] = {
	[VPU_HLOS_IOMMU_DOMAIN] = {
		.client_name = "vpu_nonsecure",
		.ctx_name = "vpu_hlos",
		.domain_num = -1,
		.is_secure = 0,
		.partitions = {
			{
				.start = SZ_128K,
				.size = SZ_1G - SZ_128K,
			},
		},
		.npartitions = 1,
	},

	[VPU_CP_IOMMU_DOMAIN] = {
		.client_name = "vpu_secure",
		.ctx_name = "vpu_cp",
		.domain_num = -1,
		.is_secure = 1,
		.partitions = {
			{
				.start = SZ_1G,
				.size = SZ_1G,
			},
		},
		.npartitions = 1,
	},

	[VPU_FW_IOMMU_DOMAIN] = {
		.client_name = "vpu_firmware",
		.ctx_name = "vpu_fw",
		.domain_num = -1,
		.is_secure = 1,
		.partitions = {
			{
				.start = 0,
				.size = SZ_16M,
			},
		},
		.npartitions = 1,
	},
};

static const char * const clk_table_dt_entries[] = {
	[VPU_BUS_CLK] = "qcom,bus-clk-load-freq-tbl",
	[VPU_MAPLE_CLK] = "qcom,maple-clk-load-freq-tbl",
	[VPU_VDP_CLK] = "qcom,vdp-clk-load-freq-tbl",
};

struct bus_pdata_config {
	int masters[2];
	int slaves[2];
	char *name;
};

static struct bus_pdata_config bus_pdata_config = {
	.masters = {MSM_BUS_MASTER_VPU, MSM_BUS_MASTER_VPU},
	.slaves = {MSM_BUS_SLAVE_EBI_CH0, MSM_BUS_SLAVE_EBI_CH0},
	.name = "qcom,bus-load-vector-tbl",
};

static size_t __get_u32_array_num_elements(struct platform_device *pdev,
		const char *name, u32 element_width)
{
	struct device_node *np = pdev->dev.of_node;
	int len_bytes = 0;
	size_t num_elements = 0;
	if (!of_get_property(np, name, &len_bytes))
		return 0;

	num_elements = len_bytes / (sizeof(u32) * element_width);
	return num_elements;
}

static int __vpu_load_freq_table(struct vpu_platform_resources *res,
		const char *name, struct load_freq_table *clk_table)
{
	int ret = 0, i;
	int num_elements = 0;
	struct platform_device *pdev = res->pdev;

	num_elements = __get_u32_array_num_elements(pdev, name, 2);
	if (num_elements == 0) {
		dprintk(VPU_INFO, "%s, no valid %s table\n", __func__, name);
		return ret;
	}

	clk_table->entry = devm_kzalloc(&pdev->dev,
		num_elements * sizeof(*clk_table->entry), GFP_KERNEL);
	if (!clk_table->entry) {
		dprintk(VPU_ERR, "%s Failed alloc load_freq_tabll\n", __func__);
		return -ENOMEM;
	}

	if (of_property_read_u32_array(pdev->dev.of_node, name,
			(u32 *) clk_table->entry, num_elements * 2)) {
		dprintk(VPU_ERR, "Failed to read frequency table\n");
		return -EINVAL;
	}

	for (i = 0; i < num_elements; i++) {
		dprintk(VPU_DBG, "%s, entry %d: load = %d, freq = %d\n",
			name, i, clk_table->entry[i].load,
			 clk_table->entry[i].freq);
	}

	clk_table->count = num_elements;
	return ret;
}

static int __vpu_load_freq_tables(struct vpu_platform_resources *res)
{
	int ret = 0, i;

	for (i = 0; i < ARRAY_SIZE(clk_table_dt_entries); i++) {
		ret = __vpu_load_freq_table(res, clk_table_dt_entries[i],
				&res->clock_tables[i]);
		if (ret)
			return ret;
	}
	return ret;
}

static int __vpu_load_bus_vector_data(struct vpu_platform_resources *res,
		int num_elements, int num_ports)
{
	struct bus_vector {
		u32 load;
		u32 ab;
		u32 ib;
	};
	struct platform_device *pdev = res->pdev;
	struct msm_bus_scale_pdata *bus_pdata = &res->bus_pdata;
	struct bus_vector *vectors; /* temporary store of bus load data */
	int i, j;

	vectors = devm_kzalloc(&pdev->dev, sizeof(*vectors) * num_elements,
				GFP_KERNEL);
	if (!vectors) {
		dprintk(VPU_ERR, "%s Failed to alloc bus_vectors\n", __func__);
		return -ENOMEM;
	}

	if (of_property_read_u32_array(pdev->dev.of_node,
	    bus_pdata_config.name, (u32 *)vectors,
	    num_elements * (sizeof(*vectors)/sizeof(u32)))) {
		dprintk(VPU_ERR, "%s Failed to read bus values\n", __func__);
		return -EINVAL;
	}

	bus_pdata->name = bus_pdata_config.name;
	bus_pdata->num_usecases = num_elements;

	bus_pdata->usecase = devm_kzalloc(&pdev->dev,
		sizeof(*bus_pdata->usecase) * num_elements, GFP_KERNEL);
	if (!bus_pdata->usecase) {
		dprintk(VPU_ERR, "%s Failed to alloc bus_pdata usecase\n",
				__func__);
		return -ENOMEM;
	}

	for (i = 0; i < bus_pdata->num_usecases; i++) {
		bus_pdata->usecase[i].vectors = devm_kzalloc(&pdev->dev,
			sizeof(*bus_pdata->usecase[i].vectors) * num_ports,
			GFP_KERNEL);
		if (!bus_pdata->usecase[i].vectors) {
			dprintk(VPU_ERR, "%s Failed to alloc usecase vectors\n",
				__func__);
			return -ENOMEM;
		}

		res->bus_table.loads[i] = vectors[i].load;

		for (j = 0; j < num_ports; j++) {
			bus_pdata->usecase[i].vectors[j].ab =
					(u64)vectors[i].ab * 1000;
			bus_pdata->usecase[i].vectors[j].ib =
					(u64)vectors[i].ib * 1000;
			bus_pdata->usecase[i].vectors[j].src =
					bus_pdata_config.masters[j];
			bus_pdata->usecase[i].vectors[j].dst =
					bus_pdata_config.slaves[j];
			dprintk(VPU_DBG,
				"load=%d, ab=%llu, ib=%llu, src=%d, dst=%d\n",
				res->bus_table.loads[i],
				bus_pdata->usecase[i].vectors[j].ab,
				bus_pdata->usecase[i].vectors[j].ib,
				bus_pdata->usecase[i].vectors[j].src,
				bus_pdata->usecase[i].vectors[j].dst);
		}
		bus_pdata->usecase[i].num_paths = num_ports;
	}

	devm_kfree(&pdev->dev, vectors);
	return 0;
}

static int __vpu_load_bus_vectors(struct vpu_platform_resources *res)
{
	int ret = 0;
	int num_elements = 0;
	struct platform_device *pdev = res->pdev;

	num_elements = __get_u32_array_num_elements(pdev,
			bus_pdata_config.name, 3);
	if (num_elements == 0) {
		dprintk(VPU_WARN, "no elements in %s\n", bus_pdata_config.name);
		return ret;
	}

	res->bus_table.count = num_elements;
	res->bus_table.loads = devm_kzalloc(&pdev->dev,
		sizeof(*res->bus_table.loads) * num_elements, GFP_KERNEL);
	if (!res->bus_table.loads) {
		dprintk(VPU_ERR, "%s, Failed to alloc memory\n", __func__);
		return -ENOMEM;
	}

	ret = __vpu_load_bus_vector_data(res, num_elements, 1);
	if (ret) {
		dprintk(VPU_ERR, "Failed to load bus vector data\n");
		return ret;
	}

	return 0;
}

static int __vpu_load_iommu_maps(struct vpu_platform_resources *res)
{
	int ret = 0, i, j, num_elements;
	struct platform_device *pdev = res->pdev;
	const char *name;
	u32 count = 0;

	num_elements = of_property_count_strings(pdev->dev.of_node,
			"qcom,enabled-iommu-maps");
	if (num_elements <= 0) {
		dprintk(VPU_WARN, "No list of IOMMUs to be enabled\n");
		return ret;
	} else if (num_elements > VPU_MAX_IOMMU_DOMAIN) {
		dprintk(VPU_ERR, "List of IOMMUs to enable is too large\n");
		return -EINVAL;
	}

	for (i = 0; i < num_elements; i++) {
		ret = of_property_read_string_index(pdev->dev.of_node,
				"qcom,enabled-iommu-maps", i, &name);
		if (ret)
			return ret;

		for (j = 0; j < VPU_MAX_IOMMU_DOMAIN; j++) {
			if (strcmp(name, iommus_array[j].client_name) == 0) {
				iommus_array[j].enabled = 1;
				count++;
			}
		}
	}

	if (!count) {
		dprintk(VPU_ERR, "No valid IOMMU names specified\n");
		return -EINVAL;
	}

	/* when these values are set, indicates that some IOMMUs are enabled */
	res->iommu_set.count = VPU_MAX_IOMMU_DOMAIN;
	res->iommu_set.iommu_maps = iommus_array;
	return 0;
}

int read_vpu_platform_resources(struct vpu_platform_resources *res,
		struct platform_device *pdev)
{
	int ret = 0;
	struct resource *kres = NULL;
	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	if (!res || !pdev)
		return -EINVAL;
	res->pdev = pdev;

	if (!pdev->dev.of_node) {
		dprintk(VPU_ERR, "%s, Device Tree node not found\n", __func__);
		return -ENOENT;
	}

	kres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res->register_base_phy = kres ? kres->start : -1;
	res->register_size = kres ? (kres->end + 1 - kres->start) : -1;

	kres = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	res->mem_base_phy = kres ? kres->start : -1;
	res->mem_size = kres ? (kres->end + 1 - kres->start) : -1;

	kres = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
			"maple_wdog_irq");
	res->irq_wd = kres ? kres->start : -1;

	kres = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
			"maple_apps_irq");
	res->irq = kres ? kres->start : -1;

	if ((res->register_base_phy | res->register_size |
			res->mem_base_phy | res->mem_size |
			res->irq_wd | res->irq) < 0) {
		dprintk(VPU_ERR,
			"%s, Failed to read platform resources\n", __func__);
		return -ENODEV;
	}
	dprintk(VPU_DBG, "%s: CSR base = 0x%08x, size = 0x%x\n", __func__,
		(u32) res->register_base_phy, res->register_size);
	dprintk(VPU_DBG, "%s: Shared mem base = 0x%08x, size = 0x%x\n",
			__func__, (u32) res->mem_base_phy, res->mem_size);
	dprintk(VPU_DBG, "%s: Wdog IRQ = %d\n", __func__, res->irq_wd);
	dprintk(VPU_DBG, "%s: IPC IRQ = %d\n", __func__, res->irq);

	/*
	 * start devres group.
	 * 'read_vpu_platform_resources' is group identifier
	 */
	if (!devres_open_group(&pdev->dev, read_vpu_platform_resources,
			GFP_KERNEL))
		return -ENOMEM;

	ret = __vpu_load_freq_tables(res);
	if (ret) {
		dprintk(VPU_ERR, "Failed to load freq tables: %d\n", ret);
		goto err_read_dt_resources;
	}
	ret = __vpu_load_bus_vectors(res);
	if (ret) {
		dprintk(VPU_ERR, "Failed to load bus vectors: %d\n", ret);
		goto err_read_dt_resources;
	}
	ret = __vpu_load_iommu_maps(res);
	if (ret) {
		dprintk(VPU_ERR, "Failed to load iommu maps: %d\n", ret);
		goto err_read_dt_resources;
	}

	/*no errors, close devres group*/
	devres_close_group(&pdev->dev, read_vpu_platform_resources);
	return 0;

err_read_dt_resources:
	free_vpu_platform_resources(res);
	return ret;
}

void free_vpu_platform_resources(struct vpu_platform_resources *res)
{
	/* free all allocations in devres group*/
	devres_release_group(&res->pdev->dev, read_vpu_platform_resources);
	memset(res, 0 , sizeof(*res));
}


/*
 * IOMMU memory management
 */

enum iommu_mem_type {
	MEM_ION,
};

enum iommu_mem_prop {
	MEM_CACHED = ION_FLAG_CACHED,
	MEM_SECURE = ION_FLAG_SECURE,
};

struct vpu_iommu_client {
	int mem_type;	/*ION memory or other*/
	void *clnt;
	struct vpu_platform_resources *res;
};

struct vpu_iommu_handle {
	/* iommu context information */
	int domain_num;
	/* mapped iommu address */
	u32 device_addr;
	/* memory buffer information */
	unsigned long size;
	unsigned long flags;
	/* memory management (ion) objects */
	struct vpu_iommu_client *iommu_client;
	void *hndl;
};

/* checks that IOMMUs are enabled. Allows debugging with contiguous memory */
static inline int __is_iommu_present(struct vpu_platform_resources *res)
{
	if (res)
		return (res->iommu_set.count > 0 &&
			res->iommu_set.iommu_maps != NULL);
	else
		return 0;
}

static int __get_iommu_domain_number(struct vpu_platform_resources *res,
		unsigned long flags, int *domain_num)
{
	struct vpu_iommu_map *iommu_map;
	int i;
	bool is_secure = (flags & MEM_SECURE) ? 1 : 0;

	*domain_num = -1;

	for (i = 0; i < res->iommu_set.count; i++) {
		iommu_map = &res->iommu_set.iommu_maps[i];
		if (iommu_map->enabled && iommu_map->is_secure == is_secure) {
			*domain_num = iommu_map->domain_num;
			return 0;
		}
	}
	return -ENOENT;
}

void *__vpu_iommu_create_client(struct vpu_platform_resources *res,
		enum iommu_mem_type type)
{
	struct vpu_iommu_client *iommu_client;
	struct ion_client *ion_client;

	iommu_client = kzalloc(sizeof(*iommu_client), GFP_KERNEL);
	if (!iommu_client) {
		dprintk(VPU_ERR, "%s, memory allocation failed\n", __func__);
		return NULL;
	}

	ion_client = msm_ion_client_create(-1, "VPU");
	if (IS_ERR(ion_client)) {
		dprintk(VPU_ERR, "%s, ION client creation failed\n", __func__);
		kfree(iommu_client);
		return NULL;
	}

	iommu_client->mem_type = type;
	iommu_client->res = res;
	iommu_client->clnt = (void *) ion_client;
	return (void *) iommu_client;
}

void __vpu_iommu_destroy_client(void *iommu_client)
{
	struct vpu_iommu_client *client =
			(struct vpu_iommu_client *) iommu_client;
	if (!client)
		return;

	ion_client_destroy((struct ion_client *) client->clnt);
	kfree(client);
}

void *vpu_iommu_create_handle(struct vpu_platform_resources *res)
{
	struct vpu_iommu_handle *handle;
	if (!res || !res->iommu_client)
		return NULL;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return NULL;

	handle->iommu_client = (struct vpu_iommu_client *) res->iommu_client;
	return handle;
}

static void __vpu_iommu_release_handle(void *iommu_handle)
{
	struct vpu_iommu_handle *handle =
			(struct vpu_iommu_handle *) iommu_handle;
	struct ion_handle *ion_handle;
	struct ion_client *ion_client;
	if (!handle)
		return;

	ion_handle = (struct ion_handle *) handle->hndl;
	ion_client = (struct ion_client *) handle->iommu_client->clnt;

	if (handle->device_addr && ion_handle) {
#ifndef VPU_V4L2_DUMMY_TEST
		if (__is_iommu_present(handle->iommu_client->res))
			ion_unmap_iommu(ion_client, ion_handle,
				handle->domain_num, 0);

		if (handle->flags & MEM_SECURE)
			if (msm_ion_unsecure_buffer(ion_client, ion_handle))
				dprintk(VPU_WARN,
					"Failed to unsecure memory\n");
#else
		ion_unmap_kernel(ion_client, ion_handle);
#endif

		handle->domain_num = -1;
		handle->device_addr = 0;
		handle->size = 0;
		handle->flags = 0;
	}

	if (ion_handle) {
		ion_free(ion_client, ion_handle);
		handle->hndl = 0;
	}
}

static int __vpu_iommu_map_handle(struct vpu_iommu_handle *handle, bool secure)
{
	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
	int ret = 0;
	unsigned long align = SZ_4K;
	bool secure_flag;
	if (!handle || !handle->hndl)
		return -EINVAL;
	ion_client = handle->iommu_client->clnt;
	ion_handle = handle->hndl;

#ifndef VPU_V4L2_DUMMY_TEST
	ret = ion_handle_get_flags(ion_client, ion_handle, &handle->flags);
	if (ret) {
		dprintk(VPU_ERR, "Failed to get ion flags: %d\n", ret);
		return ret;
	}

	/* check buffer flags */
	secure = secure ? true : false;
	secure_flag = (handle->flags & MEM_SECURE) ? true : false;
	if (secure != secure_flag) {
		dprintk(VPU_ERR, "Buffer CP status does not match request\n");
		return -EINVAL;
	}

	if (__is_iommu_present(handle->iommu_client->res)) {

		ret = __get_iommu_domain_number(handle->iommu_client->res,
				handle->flags, &handle->domain_num);
		if (ret) {
			dprintk(VPU_ERR, "Failed to get iommu domain\n");
			return ret;
		}

		if (handle->flags & MEM_SECURE) { /*handle secure buffers */
			align = SZ_1M;
			ret = msm_ion_secure_buffer(ion_client, ion_handle,
					VIDEO_PIXEL, 0);
			if (ret) {
				dprintk(VPU_ERR, "Failed to secure memory\n");
				return ret;
			}
		}

		dprintk(VPU_DBG, "%s, Using IOMMU mapping\n", __func__);
		ret = ion_map_iommu(ion_client, ion_handle,
			handle->domain_num, 0, align, 0,
			(ion_phys_addr_t *)&handle->device_addr, &handle->size,
			0 , 0);
	} else {
		dprintk(VPU_DBG, "%s, Using physical mem address\n", __func__);
		ret = ion_phys(ion_client, ion_handle,
				(ion_phys_addr_t *)&handle->device_addr,
				(size_t *)&handle->size);
	}

	if (ret) {
		dprintk(VPU_ERR, "Failed to map ion buffer\n");
		if (handle->flags & MEM_SECURE)
			msm_ion_unsecure_buffer(ion_client, ion_handle);
		return ret;
	}
#else
	(void)__get_iommu_domain_number; /* quiet the compiler */
	(void)align; /* quiet the compiler */
	(void)ret; /* quiet the compiler */
	handle->device_addr = (u32) ion_map_kernel(ion_client, ion_handle);
	if (IS_ERR_OR_NULL((void *)handle->device_addr)) {
		dprintk(VPU_ERR, "Failed to map ion buffer to kernel\n");
		return -ENOMEM;
	}
#endif

	return 0;
}

int vpu_iommu_map_user_mem(void *iommu_handle, unsigned long fd,
		unsigned long length, unsigned long offset, bool secure)
{
	struct vpu_iommu_handle *handle =
			(struct vpu_iommu_handle *) iommu_handle;
	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
	int ret = 0;
	if (!handle)
		return -EINVAL;

	if (handle->device_addr)
		__vpu_iommu_release_handle(iommu_handle);

	ion_client = handle->iommu_client->clnt;
	ion_handle = ion_import_dma_buf(ion_client, fd);
	if (IS_ERR_OR_NULL(ion_handle)) {
		dprintk(VPU_ERR, "%s, ION import failed with %ld\n",
				__func__, PTR_ERR(ion_handle));
		return -ENOMEM;
	}
	handle->hndl = ion_handle;

	ret = __vpu_iommu_map_handle(handle, secure);
	if (ret) {
		dprintk(VPU_ERR, "%s, iommu memory mapping failed\n", __func__);
		goto err_map_handle;
	}

	if (handle->size < length || handle->size < offset ||
			handle->size < (offset + length)) {
		dprintk(VPU_ERR, "%s, mapped buffer is too small\n", __func__);
		__vpu_iommu_release_handle(iommu_handle);
		return -ENOMEM;
	}
	handle->device_addr += offset;
	handle->size = length;
	return 0;

err_map_handle:
	ion_free(ion_client, ion_handle);
	handle->hndl = 0;
	return ret;
}

int vpu_iommu_alloc_kernel_mem(void *iommu_handle, unsigned long size,
		bool secure)
{
	struct vpu_iommu_handle *handle =
			(struct vpu_iommu_handle *) iommu_handle;
	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
	unsigned long align, flags;
	unsigned long heap_mask;
	int ret = 0;
	if (!handle || !size)
		return -EINVAL;

	if (handle->device_addr)
		__vpu_iommu_release_handle(iommu_handle);

	ion_client = handle->iommu_client->clnt;
	flags = secure ? MEM_SECURE : 0;

	if (!__is_iommu_present(handle->iommu_client->res)) {
		align = SZ_4K;
		heap_mask = ION_HEAP(ION_SYSTEM_CONTIG_HEAP_ID);
	} else {
		heap_mask = ION_HEAP(ION_IOMMU_HEAP_ID);
		if (flags & MEM_SECURE)
			align = SZ_1M;
		else
			align = SZ_4K;
	}
	handle->size = ALIGN(size, align);

	ion_handle = ion_alloc(ion_client, handle->size, align,
			heap_mask, flags);
	if (IS_ERR_OR_NULL(ion_handle)) {
		dprintk(VPU_ERR, "%s, ION alloc failed with %ld\n",
				__func__, PTR_ERR(ion_handle));
		ret = -ENOMEM;
		goto err_alloc_handle;
	}
	handle->hndl = ion_handle;

	ret = __vpu_iommu_map_handle(handle, secure);
	if (ret) {
		dprintk(VPU_ERR, "%s, iommu memory mapping failed\n", __func__);
		goto err_map_handle;
	}

	dprintk(VPU_DBG, "%s, ION alloc success. Addr = %p size = 0x%08x\n",
		__func__, (void *) handle->device_addr, (u32) handle->size);
	return 0;

err_map_handle:
	ion_free(ion_client, ion_handle);
	handle->hndl = 0;
err_alloc_handle:
	handle->size = 0;
	return ret;
}

void vpu_iommu_destroy_handle(void *iommu_handle)
{
	if (!iommu_handle)
		return;
	__vpu_iommu_release_handle(iommu_handle);
	kfree(iommu_handle);
}

u32 vpu_iommu_get_addr(void *iommu_handle)
{
	struct vpu_iommu_handle *handle =
			(struct vpu_iommu_handle *) iommu_handle;
	if (!handle || !handle->hndl)
		return 0;
	else
		return handle->device_addr;
}

u32 vpu_iommu_get_size(void *iommu_handle)
{
	struct vpu_iommu_handle *handle =
			(struct vpu_iommu_handle *) iommu_handle;
	if (!handle || !handle->hndl)
		return 0;
	else
		return handle->size;
}

int register_vpu_iommu_domains(struct vpu_platform_resources *res)
{
	int i = 0, ret = -EINVAL;
	struct vpu_iommu_map *iommu_map;
	struct msm_iova_layout layout;

	if (!res)
		return ret;

	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	for (i = 0; i < res->iommu_set.count; i++) {
		iommu_map = &res->iommu_set.iommu_maps[i];
		if (!iommu_map->enabled)
			continue;

		memset(&layout, 0, sizeof(layout));
		layout.partitions = iommu_map->partitions;
		layout.npartitions = iommu_map->npartitions;
		layout.client_name = iommu_map->client_name;
		layout.is_secure = iommu_map->is_secure;

		iommu_map->domain_num = msm_register_domain(&layout);
		if (iommu_map->domain_num < 0) {
			dprintk(VPU_ERR, "IOMMU domain %d register fail\n", i);
			goto fail_group;
		}

		iommu_map->domain = msm_get_iommu_domain(iommu_map->domain_num);
		if (!iommu_map->domain) {
			dprintk(VPU_ERR, "Failed to get IOMMU domain %d\n", i);
			goto fail_group;
		}

		iommu_map->ctx = msm_iommu_get_ctx(iommu_map->ctx_name);
		if (IS_ERR_OR_NULL(iommu_map->ctx)) {
			if (PTR_ERR(iommu_map->ctx) == -EPROBE_DEFER) {
				dprintk(VPU_INFO,
					"EPROBE_DEFER from iommu_get_ctx: %s\n",
					iommu_map->ctx_name);
				ret = -EPROBE_DEFER;
			} else {
				dprintk(VPU_ERR, "failed iommu_get_ctx: %s\n",
					iommu_map->ctx_name);
			}
			goto fail_group;
		}

		dprintk(VPU_DBG, "%s, iommu %d: %s domain_number = %d\n",
			__func__, i, iommu_map->client_name,
			iommu_map->domain_num);
	}

	/* Create VPU device iommu (ion) client */
	res->iommu_client = __vpu_iommu_create_client(res, MEM_ION);
	if (!res->iommu_client) {
		dprintk(VPU_ERR, "could not create iommu client\n");
		ret = -ENOMEM;
		goto fail_group;
	}

	return 0;

fail_group:
	unregister_vpu_iommu_domains(res);
	return ret;
}

void unregister_vpu_iommu_domains(struct vpu_platform_resources *res)
{
	struct vpu_iommu_map *iommu_map;
	int i = 0;
	if (!res)
		return;
	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	detach_vpu_iommus(res); /* if not already detached */

	if (res->iommu_client) {
		__vpu_iommu_destroy_client(res->iommu_client);
		res->iommu_client = NULL;
	}

	for (i = 0; i < res->iommu_set.count; i++) {
		iommu_map = &res->iommu_set.iommu_maps[i];
		if (iommu_map->domain)
			msm_unregister_domain(iommu_map->domain);
		iommu_map->domain = NULL;
		iommu_map->domain_num = -1;
		iommu_map->ctx = NULL;
	}
}

int attach_vpu_iommus(struct vpu_platform_resources *res)
{
	int i, ret;
	struct vpu_iommu_map *iommu_map;
	if (!res)
		return -EINVAL;

	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	for (i = 0; i < res->iommu_set.count; i++) {
		iommu_map = &res->iommu_set.iommu_maps[i];
		if (!iommu_map->enabled || iommu_map->attached)
			continue;

		ret = iommu_attach_device(iommu_map->domain, iommu_map->ctx);
		if (ret) {
			dprintk(VPU_ERR, "Failed to attach IOMMU device %s\n",
					iommu_map->ctx_name);
			goto fail_group;
		}
		iommu_map->attached = 1;
	}
	return 0;

fail_group:
	detach_vpu_iommus(res);
	return ret;
}

void detach_vpu_iommus(struct vpu_platform_resources *res)
{
	struct vpu_iommu_map *iommu_map;
	int i = 0;
	if (!res)
		return;
	dprintk(VPU_DBG, "Enter function %s\n", __func__);

	for (i = 0; i < res->iommu_set.count; i++) {
		iommu_map = &res->iommu_set.iommu_maps[i];
		if (iommu_map->attached)
			iommu_detach_device(iommu_map->domain, iommu_map->ctx);
		iommu_map->attached = 0;
	}
}
