#ifndef _LINUX_SYSINFO_H
#define _LINUX_SYSINFO_H

#include <linux/types.h>

#define SI_LOAD_SHIFT	16
struct sysinfo {
	/* Seconds since boot */
	__KERNEL_NATIVE_LONG_TYPE uptime;
	/* 1, 5, and 15 minute load averages */
	unsigned __KERNEL_NATIVE_LONG_TYPE loads[3];
	/* Total usable main memory size */
	unsigned __KERNEL_NATIVE_LONG_TYPE totalram;
	/* Available memory size */
	unsigned __KERNEL_NATIVE_LONG_TYPE freeram;
	/* Amount of shared memory */
	unsigned __KERNEL_NATIVE_LONG_TYPE sharedram;
	/* Memory used by buffers */
	unsigned __KERNEL_NATIVE_LONG_TYPE bufferram;
	/* Total swap space size */
	unsigned __KERNEL_NATIVE_LONG_TYPE totalswap;
	/* swap space still available */
	unsigned __KERNEL_NATIVE_LONG_TYPE freeswap;
	/* Number of current processes */
	unsigned short procs;
	/* explicit padding for m68k */
	unsigned short pad;
	/* Total high memory size */
	unsigned __KERNEL_NATIVE_LONG_TYPE totalhigh;
	/* Available high memory size */
	unsigned __KERNEL_NATIVE_LONG_TYPE freehigh;
	/* Memory unit size in bytes */
	unsigned int mem_unit;
	/* Padding: libc5 uses this.. */
	char _f[20-2*sizeof(__KERNEL_NATIVE_LONG_TYPE)-sizeof(int)];
};

#endif
