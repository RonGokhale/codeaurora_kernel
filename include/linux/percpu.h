#ifndef __LINUX_PERCPU_H
#define __LINUX_PERCPU_H

#include <linux/mmdebug.h>
#include <linux/preempt.h>
#include <linux/smp.h>
#include <linux/cpumask.h>
#include <linux/pfn.h>
#include <linux/init.h>
#include <linux/workqueue.h>

#include <asm/percpu.h>

/* enough to cover all DEFINE_PER_CPUs in modules */
#ifdef CONFIG_MODULES
#define PERCPU_MODULE_RESERVE		(8 << 10)
#else
#define PERCPU_MODULE_RESERVE		0
#endif

#ifndef PERCPU_ENOUGH_ROOM
#define PERCPU_ENOUGH_ROOM						\
	(ALIGN(__per_cpu_end - __per_cpu_start, SMP_CACHE_BYTES) +	\
	 PERCPU_MODULE_RESERVE)
#endif

/* minimum unit size, also is the maximum supported allocation size */
#define PCPU_MIN_UNIT_SIZE		PFN_ALIGN(32 << 10)

/*
 * Percpu allocator can serve percpu allocations before slab is
 * initialized which allows slab to depend on the percpu allocator.
 * The following two parameters decide how much resource to
 * preallocate for this.  Keep PERCPU_DYNAMIC_RESERVE equal to or
 * larger than PERCPU_DYNAMIC_EARLY_SIZE.
 */
#define PERCPU_DYNAMIC_EARLY_SLOTS	128
#define PERCPU_DYNAMIC_EARLY_SIZE	(12 << 10)

/*
 * PERCPU_DYNAMIC_RESERVE indicates the amount of free area to piggy
 * back on the first chunk for dynamic percpu allocation if arch is
 * manually allocating and mapping it for faster access (as a part of
 * large page mapping for example).
 *
 * The following values give between one and two pages of free space
 * after typical minimal boot (2-way SMP, single disk and NIC) with
 * both defconfig and a distro config on x86_64 and 32.  More
 * intelligent way to determine this would be nice.
 */
#if BITS_PER_LONG > 32
#define PERCPU_DYNAMIC_RESERVE		(20 << 10)
#else
#define PERCPU_DYNAMIC_RESERVE		(12 << 10)
#endif

extern void *pcpu_base_addr;
extern const unsigned long *pcpu_unit_offsets;

struct pcpu_group_info {
	int			nr_units;	/* aligned # of units */
	unsigned long		base_offset;	/* base address offset */
	unsigned int		*cpu_map;	/* unit->cpu map, empty
						 * entries contain NR_CPUS */
};

struct pcpu_alloc_info {
	size_t			static_size;
	size_t			reserved_size;
	size_t			dyn_size;
	size_t			unit_size;
	size_t			atom_size;
	size_t			alloc_size;
	size_t			__ai_size;	/* internal, don't use */
	int			nr_groups;	/* 0 if grouping unnecessary */
	struct pcpu_group_info	groups[];
};

enum pcpu_fc {
	PCPU_FC_AUTO,
	PCPU_FC_EMBED,
	PCPU_FC_PAGE,

	PCPU_FC_NR,
};
extern const char * const pcpu_fc_names[PCPU_FC_NR];

extern enum pcpu_fc pcpu_chosen_fc;

typedef void * (*pcpu_fc_alloc_fn_t)(unsigned int cpu, size_t size,
				     size_t align);
typedef void (*pcpu_fc_free_fn_t)(void *ptr, size_t size);
typedef void (*pcpu_fc_populate_pte_fn_t)(unsigned long addr);
typedef int (pcpu_fc_cpu_distance_fn_t)(unsigned int from, unsigned int to);

extern struct pcpu_alloc_info * __init pcpu_alloc_alloc_info(int nr_groups,
							     int nr_units);
extern void __init pcpu_free_alloc_info(struct pcpu_alloc_info *ai);

extern int __init pcpu_setup_first_chunk(const struct pcpu_alloc_info *ai,
					 void *base_addr);

#ifdef CONFIG_NEED_PER_CPU_EMBED_FIRST_CHUNK
extern int __init pcpu_embed_first_chunk(size_t reserved_size, size_t dyn_size,
				size_t atom_size,
				pcpu_fc_cpu_distance_fn_t cpu_distance_fn,
				pcpu_fc_alloc_fn_t alloc_fn,
				pcpu_fc_free_fn_t free_fn);
#endif

#ifdef CONFIG_NEED_PER_CPU_PAGE_FIRST_CHUNK
extern int __init pcpu_page_first_chunk(size_t reserved_size,
				pcpu_fc_alloc_fn_t alloc_fn,
				pcpu_fc_free_fn_t free_fn,
				pcpu_fc_populate_pte_fn_t populate_pte_fn);
#endif

extern void __percpu *__alloc_reserved_percpu(size_t size, size_t align);
extern bool is_kernel_percpu_address(unsigned long addr);

#if !defined(CONFIG_SMP) || !defined(CONFIG_HAVE_SETUP_PER_CPU_AREA)
extern void __init setup_per_cpu_areas(void);
#endif
extern void __init percpu_init_late(void);

extern void __percpu *__alloc_percpu(size_t size, size_t align);
extern void free_percpu(void __percpu *__pdata);
extern phys_addr_t per_cpu_ptr_to_phys(void *addr);

#define alloc_percpu(type)	\
	(typeof(type) __percpu *)__alloc_percpu(sizeof(type), __alignof__(type))

/*
 * percpu_pool is an automatically managed percpu allocation cache which
 * can be used to allocate percpu areas from atomic contexts.
 */
struct percpu_pool {
	spinlock_t		lock;
	size_t			elem_size;
	size_t			elem_align;
	int			nr_low;
	int			nr_high;
	int			nr;
	bool			inhibit_fill:1;
	void __percpu		*head;
	struct work_struct	fill_work;
};

void __pcpu_pool_fill_workfn(struct work_struct *work);

/**
 * __PERCPU_POOL_INIT - initializer for percpu_pool
 * @name: name of the percpu_pool being initialized
 * @size: size of the percpu elements to be cached
 * @align: alignment of the percpu elements to be cached
 * @low: low watermark of the pool
 * @high: high watermark of the pool
 *
 * Initializer for percpu_pool @name which serves percpu areas of @size
 * bytes with the alignment of @align.  If the pool falls below @low, it's
 * filled upto @high.  Note that the pool starts empty.  If not explicitly
 * filled with percpu_pool_fill(), the first allocation will fail and
 * trigger filling.
 */
#define __PERCPU_POOL_INIT(name, size, align, low, high)		\
{									\
	.lock			= __SPIN_LOCK_INITIALIZER(name.lock),	\
	.elem_size		= (size),				\
	.elem_align		= (align),				\
	.nr_low			= (low),				\
	.nr_high		= (high),				\
	.fill_work		= __WORK_INITIALIZER(name.fill_work,	\
					__pcpu_pool_fill_workfn),	\
}

/**
 * __DEFINE_PERCPU_POOL - define a percpu_pool
 * @name: name of the percpu_pool being defined
 * @size: size of the percpu elements to be cached
 * @align: alignment of the percpu elements to be cached
 * @low: low watermark of the pool
 * @high: high watermark of the pool
 *
 * Define a percpu_pool @name.  See __PERCPU_POOL_INIT().
 */
#define __DEFINE_PERCPU_POOL(name, size, align, low, high)		\
	struct percpu_pool name = __PERCPU_POOL_INIT(name, size, align,	\
						     low, high)

/**
 * PERCPU_POOL_INIT - initializer for percpu_pool
 * @name: name of the percpu_pool being initialized
 * @type: type of the percpu elements to be cached
 * @low: low watermark of the pool
 * @high: high watermark of the pool
 *
 * Equivalent to __PERCPU_POOL_INIT() except that the size and alignment
 * are calculated from @type instead of being explicitly specified.
 */
#define PERCPU_POOL_INIT(name, type, low, high)				\
	__PERCPU_POOL_INIT(name, sizeof(type), __alignof__(type),	\
			   low, high)

/**
 * DEFINE_PERCPU_POOL - define a percpu_pool
 * @name: name of the percpu_pool being defined
 * @type: type of the percpu elements to be cached
 * @low: low watermark of the pool
 * @high: high watermark of the pool
 *
 * Equivalent to __DEFINE_PERCPU_POOL() except that the size and alignment
 * are calculated from @type instead of being explicitly specified.
 */
#define DEFINE_PERCPU_POOL(name, type, low, high)			\
	__DEFINE_PERCPU_POOL(name, sizeof(type), __alignof__(type),	\
			     low, high)

extern void percpu_pool_fill(struct percpu_pool *pool, int target_nr);
extern void percpu_pool_empty(struct percpu_pool *pool);
extern void __percpu *percpu_pool_alloc(struct percpu_pool *pool);

#endif /* __LINUX_PERCPU_H */
