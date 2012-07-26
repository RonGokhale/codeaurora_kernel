/*
	All files except if stated otherwise in the begining of the file are under the ISC license:
	-----------------------------------------------------------------------------------

	Copyright (c) 2010-2012 Design Art Networks Ltd.

	Permission to use, copy, modify, and/or distribute this software for any
	purpose with or without fee is hereby granted, provided that the above
	copyright notice and this permission notice appear in all copies.

	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/irq.h>

#include "danipc_k.h"


#include "I_sys_types.h"
#include "IPC_api.h"
#include "I_phy_addr_space.h"

#include "danipc_lowlevel.h"

#include "glob_mem.c"

/* FIXME: take from IPC_reg.h */
#define dan_ipc_if_CDU_INT0_ENABLE_OFFSET	(0x518)
#define dan_ipc_if_CDU_INT0_STATUS_OFFSET	(0x520)
#define dan_ipc_if_CDU_INT0_CLEAR_OFFSET	(0x530)

/* See IPC_trns_fifo.c, l.138 */
#define IPC_FIFO_IRQ_OFFSET	4

/* See IPC_trns_fifo.c, l.143 */
#define IPC_FIFO_RD_IN_HIGH_IRQ_MASK	(2<<(4*4))
#define IPC_IRQ_MASK			IPC_FIFO_RD_IN_HIGH_IRQ_MASK

/* IPC and Linux coexistence.
 * IPC uses/needs physical addresses with bit 31 set while Linux obviously
 * uses virtual addresses. So when writing an address to IPC / reading from IPC
 * make sure it is converted from virtual to IPC address
 * (physical address with bit 31 set) and vice versa.
 * For every cpuid (except my own) FIFO buffers of both priorities remapped.
 * It is guaranteed (?) that FIFO buffers are in contiguous memory of 16kB long.
 * So remapping of 2*16kB is a safe way to access all possible FIFO buffers.
 * For my own CPU just take physical address.
 * Vladik, 21.08.2011
 */

#define FIFO_MAP_SIZE		SZ_16K
#define FIFO_MAP_MASK		(FIFO_MAP_SIZE - 1)
#define CACHE_BIT		(1<<31)

ipc_buffer_t			*ipc_buffers;

ipc_to_virt_map_t		ipc_to_virt_map[PLATFORM_MAX_NUM_OF_NODES][2];

static void init_own_ipc_to_virt_map(void)
{
	ipc_to_virt_map_t *high_map = &ipc_to_virt_map[PLATFORM_my_ipc_id][IPC_trns_prio_e_1];
	ipc_to_virt_map_t *low_map = &ipc_to_virt_map[PLATFORM_my_ipc_id][IPC_trns_prio_e_0];

	/* This prevents remapping by remap_fifo_mem() */
	high_map->vaddr		= ipc_buffers[0];
	high_map->paddr		= virt_to_phys(high_map->vaddr);

	low_map->vaddr		= ipc_buffers[IPC_BUF_COUNT_MAX];
	low_map->paddr		= virt_to_phys(low_map->vaddr);
}


static void unmap_ipc_to_virt_map(void)
{
	int		cpuid;

	for (cpuid = 0; cpuid < PLATFORM_MAX_NUM_OF_NODES; cpuid++) {
		if (cpuid == PLATFORM_my_ipc_id)
			continue;
		if (ipc_to_virt_map[cpuid][IPC_trns_prio_e_1].vaddr)
			iounmap(ipc_to_virt_map[cpuid][IPC_trns_prio_e_1].vaddr);
		if (ipc_to_virt_map[cpuid][IPC_trns_prio_e_0].vaddr)
			iounmap(ipc_to_virt_map[cpuid][IPC_trns_prio_e_0].vaddr);
	}
}

static void remap_fifo_mem(const int cpuid, const unsigned prio, const uint32_t paddr)
{
	ipc_to_virt_map_t *const map = &ipc_to_virt_map[cpuid][prio];

	/* Round down to nearest 16kB address */
	const uint32_t		start_addr = ((paddr + FIFO_MAP_MASK) & ~FIFO_MAP_MASK) -
						2 * FIFO_MAP_SIZE;
	map->paddr		= start_addr;
	map->vaddr		= ioremap_nocache(start_addr, 2 * FIFO_MAP_SIZE);

	if ( !map->vaddr ) {
		printk("%s:%d cpuid = %d priority = %u cannot remap FIFO memory at addr. 0x%x\n",
			__func__, __LINE__, cpuid, prio, start_addr);
		BUG();
	}
}

uint32_t virt_to_ipc(const int cpuid, const unsigned prio, void *vaddr)
{
	if (prio <= IPC_trns_prio_e_1) {
		ipc_to_virt_map_t	*map = &ipc_to_virt_map[cpuid][prio];
		int		offset;

		if ( !map->paddr ) {
			printk(KERN_ERR "%s:%d: cpuid = %d priority = %u unmapped\n",
				__func__, __LINE__, cpuid, prio);
			BUG();
		}
		offset = (unsigned)vaddr - (unsigned)map->vaddr;
		return (map->paddr + offset) | CACHE_BIT;
	}
	else {
		printk(KERN_ERR "%s:%d: cpuid = %d illegal priority = %u\n",
			__func__, __LINE__, cpuid, prio);
		BUG();
	}

	return 0;
}

void *ipc_to_virt(const int cpuid, const unsigned prio, const uint32_t ipc_addr)
{
	if (prio <= IPC_trns_prio_e_1 || cpuid < PLATFORM_MAX_NUM_OF_NODES) {
		ipc_to_virt_map_t	*map = &ipc_to_virt_map[cpuid][prio];
		const uint32_t	paddr = ipc_addr & ~CACHE_BIT;
		int		offset;

		if ( !map->paddr ) {
			remap_fifo_mem(cpuid, prio, paddr);
		}
		offset = paddr - map->paddr;
		return (uint8_t*)map->vaddr + offset;
	}
	else {
		printk(KERN_ERR "%s:%d: cpuid = %d illegal priority = %u\n",
			__func__, __LINE__, cpuid, prio);
		BUG();
	}
	return NULL;
}


void high_prio_rx(unsigned long data)
{
	struct net_device	*dev = (struct net_device *)data;

	IPC_recieve(IPC_FIFO_BUF_NUM_HIGH, IPC_trns_prio_e_1);

	enable_irq(dev->irq);
}

static void danipc_irq_clear(void)
{
	const void	*base_addr = IPC_array_hw_access[PLATFORM_my_ipc_id];
	const unsigned	irq_addr = (unsigned)base_addr + IPC_FIFO_IRQ_OFFSET;

	writel(IPC_IRQ_MASK, irq_addr + dan_ipc_if_CDU_INT0_CLEAR_OFFSET);
}

irqreturn_t danipc_interrupt(int irq, void *data)
{
	struct net_device	*dev = (struct net_device *)data;
	danipc_priv_t		*priv = netdev_priv(dev);

	disable_irq_nosync(irq);
	danipc_irq_clear();

	tasklet_schedule(&priv->rx_task);

	return IRQ_HANDLED;
}

void danipc_init_irq(struct net_device *dev, danipc_priv_t *priv)
{
	const void	*base_addr = IPC_array_hw_access[PLATFORM_my_ipc_id];
	const unsigned	irq_addr = (unsigned)base_addr + IPC_FIFO_IRQ_OFFSET;
	volatile unsigned irq_stat;

	writel(IPC_IRQ_MASK, irq_addr + dan_ipc_if_CDU_INT0_ENABLE_OFFSET);
	writel(IPC_IRQ_MASK, irq_addr + dan_ipc_if_CDU_INT0_CLEAR_OFFSET);
	irq_stat = readl(irq_addr + dan_ipc_if_CDU_INT0_STATUS_OFFSET);
}


static void remap_agent_table(void)
{
	agentTable = ioremap_nocache(GLOBMEM_IPC_AGENT_BUF_ADDR,
					GLOBMEM_IPC_AGENT_BUF_SIZE);
	printk(KERN_INFO "%s: remapped agentTable @ 0x%p\n", __func__, agentTable);
}

static void unmap_agent_table(void)
{
	iounmap(agentTable);
}

static void prepare_node(const int cpuid)
{
	ipc_to_virt_map_t	*map;
	
	IPC_array_hw_access[cpuid] =
		ioremap_nocache(IPC_array_hw_access_phys[cpuid], 0x64);
	printk(KERN_INFO "%s: n=%d: 0x%x --> 0x%p\n", __func__, cpuid,
		IPC_array_hw_access_phys[cpuid], IPC_array_hw_access[cpuid]);

	map = &ipc_to_virt_map[cpuid][IPC_trns_prio_e_0];
	atomic_set(&map->pending_skbs, 0);

	map = &ipc_to_virt_map[cpuid][IPC_trns_prio_e_1];
	atomic_set(&map->pending_skbs, 0);
}

static void prepare_nodes(void)
{
	int		n;

	for (n = 0; n < PLATFORM_MAX_NUM_OF_NODES; n++)
		if (IPC_array_hw_access_phys[n] != IPC_DUMMY_ADDR)
			prepare_node(n);
}

static void unmap_nodes_memory(void)
{
	int		n;

	for (n = 0; n < PLATFORM_MAX_NUM_OF_NODES; n++)
		if (IPC_array_hw_access[n]) {
			printk(KERN_INFO "%s: n=%d: 0x%p\n", __func__,
				n, IPC_array_hw_access[n]);
			iounmap(IPC_array_hw_access[n]);
		}
}

static void remap_semaphore(void)
{
	semaphore = ioremap_nocache(NPU_SEMAPHORE0, 0x200);
}

static void unmap_semaphore(void)
{
	iounmap(semaphore);
}


static void *alloc_ipc_buffers(void)
{
	const unsigned long	order = get_order(IPC_BUF_SIZE);

	ipc_buffers = (ipc_buffer_t *)__get_free_pages(GFP_KERNEL, order);
	if (ipc_buffers)
		memset(ipc_buffers, 0, PAGE_SIZE << order);
	else 
		printk(KERN_ERR "%s:%d: cannot allocate IPC buffers!\n",
			__func__, __LINE__);
	return ipc_buffers;
}

static void free_ipc_buffers(void)
{
	if (ipc_buffers) {
		const unsigned long	order = get_order(IPC_BUF_SIZE);

		free_pages((unsigned long)ipc_buffers, order);
	}
}


int danipc_ll_init(void)
{
	int		rc = -1;

	if (alloc_ipc_buffers() != NULL) {
		prepare_nodes();
		remap_agent_table();
		remap_semaphore();
		init_own_ipc_to_virt_map();

		rc = IPC_init();
	}

	return rc;
}


void danipc_ll_cleanup(void)
{
	unmap_ipc_to_virt_map();
	unmap_semaphore();
	unmap_agent_table();
	unmap_nodes_memory();
	free_ipc_buffers();
}


void danipc_poll(struct net_device *dev)
{
	(void)dev;
	IPC_recieve(IPC_FIFO_BUF_NUM_LOW, IPC_trns_prio_e_0);
}
