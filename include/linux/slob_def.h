#ifndef __LINUX_SLOB_DEF_H
#define __LINUX_SLOB_DEF_H

static __always_inline void *kmalloc(size_t size, gfp_t flags)
{
	return __kmalloc_node(size, flags, NUMA_NO_NODE);
}

#endif /* __LINUX_SLOB_DEF_H */
