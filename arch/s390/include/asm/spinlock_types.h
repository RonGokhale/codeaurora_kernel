#ifndef __ASM_SPINLOCK_TYPES_H
#define __ASM_SPINLOCK_TYPES_H

#ifndef __LINUX_SPINLOCK_TYPES_H
# error "please don't include this file directly"
#endif

#ifdef CONFIG_S390_TICKET_SPINLOCK

typedef struct arch_spinlock {
	union {
		unsigned int lock;
		struct __raw_tickets {
			u16 owner;
			u8 tail;
			u8 head;
		} tickets;
	};
} arch_spinlock_t;

#else /* CONFIG_S390_TICKET_SPINLOCK */

typedef struct {
	unsigned int lock;
} __attribute__ ((aligned (4))) arch_spinlock_t;

#endif /* CONFIG_S390_TICKET_SPINLOCK */

#define __ARCH_SPIN_LOCK_UNLOCKED { .lock = 0, }

typedef struct {
	unsigned int lock;
} arch_rwlock_t;

#define __ARCH_RW_LOCK_UNLOCKED		{ 0 }

#endif
