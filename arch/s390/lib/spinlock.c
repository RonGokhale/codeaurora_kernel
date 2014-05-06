/*
 *    Out of line spinlock code.
 *
 *    Copyright IBM Corp. 2004, 2014
 *    Author(s): Martin Schwidefsky (schwidefsky@de.ibm.com)
 *		 Philipp Hachtmann (phacht@linux.vnet.ibm.com)
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <asm/io.h>

int spin_retry = 1000;

/**
 * spin_retry= parameter
 */
static int __init spin_retry_setup(char *str)
{
	spin_retry = simple_strtoul(str, &str, 0);
	return 1;
}
__setup("spin_retry=", spin_retry_setup);

#ifdef CONFIG_S390_TICKET_SPINLOCK

void arch_spin_lock_wait(arch_spinlock_t *lp)
{
	arch_spinlock_t cur, new;
	int cpu, owner, count;
	u8 ticket = 0;

	cpu = smp_processor_id();
	count = spin_retry;
	while (1) {
		new.lock = cur.lock = ACCESS_ONCE(lp->lock);
		if (new.lock == 0) {
			/* The lock is free with no waiter, try to get it. */
			new.tickets.owner = (u16) ~cpu;
		} else if (!ticket) {
			/* Try to get a ticket. */
			new.tickets.tail = (u8)(new.tickets.tail + 1) ? : 1;
			if (new.tickets.tail == new.tickets.head)
				/* Overflow, can't get a ticket. */
				new.tickets.tail = cur.tickets.tail;
		} else if (new.tickets.head == ticket)
			new.tickets.owner = (u16) ~cpu;
		/* Do the atomic update. */
		if (cur.lock != new.lock &&
		    _raw_compare_and_swap(&lp->lock, cur.lock, new.lock)) {
			/* Update successful. */
			if (new.tickets.owner == (u16) ~cpu)
				return;		/* Got the lock. */
			ticket = new.tickets.tail; /* Got a ticket. */
			count = 0;
		}
		/* Lock could not be acquired yet. */
		if (count--)
			continue;
		count = spin_retry;
		owner = cur.tickets.owner;
		if (ticket) {
			if (owner && smp_vcpu_scheduled(~owner)) {
				if (MACHINE_IS_LPAR)
					continue;
			} else
				count = 0;
		}
		/* Yield the cpu. */
		if (owner)
			smp_yield_cpu(~owner);
		else
			smp_yield();
	}
}
EXPORT_SYMBOL(arch_spin_lock_wait);

void arch_spin_unlock_slow(arch_spinlock_t *lp)
{
	arch_spinlock_t cur, new;

	do {
		cur.lock = ACCESS_ONCE(lp->lock);
		new.lock = 0;
		if (cur.tickets.head != cur.tickets.tail) {
			new.tickets.tail = cur.tickets.tail;
			new.tickets.head = (u8)(cur.tickets.head + 1) ? : 1;
			new.tickets.owner = 0;
		}
	} while (!_raw_compare_and_swap(&lp->lock, cur.lock, new.lock));
}
EXPORT_SYMBOL(arch_spin_unlock_slow);

void arch_spin_relax(arch_spinlock_t *lp)
{
	unsigned int cpu = lp->tickets.owner;

	if (cpu != 0) {
		if (MACHINE_IS_VM || MACHINE_IS_KVM ||
		    !smp_vcpu_scheduled(~cpu))
			smp_yield_cpu(~cpu);
	}
}
EXPORT_SYMBOL(arch_spin_relax);

#else /* CONFIG_S390_TICKET_SPINLOCK */

void arch_spin_lock_wait(arch_spinlock_t *lp)
{
	unsigned int cpu = SPINLOCK_LOCKVAL;
	unsigned int owner;
	int count;

	while (1) {
		owner = lp->lock;
		if (!owner || smp_vcpu_scheduled(~owner)) {
			count = spin_retry;
			do {
				if (arch_spin_is_locked(lp))
					continue;
				if (_raw_compare_and_swap(&lp->lock, 0, cpu))
					return;
			} while (count-- > 0);
			if (MACHINE_IS_LPAR)
				continue;
		}
		owner = lp->lock;
		if (owner)
			smp_yield_cpu(~owner);
		if (_raw_compare_and_swap(&lp->lock, 0, cpu))
			return;
	}
}
EXPORT_SYMBOL(arch_spin_lock_wait);

void arch_spin_lock_wait_flags(arch_spinlock_t *lp, unsigned long flags)
{
	unsigned int cpu = SPINLOCK_LOCKVAL;
	unsigned int owner;
	int count;

	local_irq_restore(flags);
	while (1) {
		owner = lp->lock;
		if (!owner || smp_vcpu_scheduled(~owner)) {
			count = spin_retry;
			do {
				if (arch_spin_is_locked(lp))
					continue;
				local_irq_disable();
				if (_raw_compare_and_swap(&lp->lock, 0, cpu))
					return;
				local_irq_restore(flags);
			} while (count-- > 0);
			if (MACHINE_IS_LPAR)
				continue;
		}
		owner = lp->lock;
		if (owner)
			smp_yield_cpu(~owner);
		local_irq_disable();
		if (_raw_compare_and_swap(&lp->lock, 0, cpu))
			return;
		local_irq_restore(flags);
	}
}
EXPORT_SYMBOL(arch_spin_lock_wait_flags);

void arch_spin_relax(arch_spinlock_t *lp)
{
	unsigned int cpu = lp->lock;
	if (cpu != 0) {
		if (MACHINE_IS_VM || MACHINE_IS_KVM ||
		    !smp_vcpu_scheduled(~cpu))
			smp_yield_cpu(~cpu);
	}
}
EXPORT_SYMBOL(arch_spin_relax);

#endif /* CONFIG_S390_TICKET_SPINLOCK */

int arch_spin_trylock_retry(arch_spinlock_t *lp)
{
	int count;

	for (count = spin_retry; count > 0; count--) {
		if (arch_spin_is_locked(lp))
			continue;
		if (arch_spin_trylock_once(lp))
			return 1;
	}
	return 0;
}
EXPORT_SYMBOL(arch_spin_trylock_retry);

void _raw_read_lock_wait(arch_rwlock_t *rw)
{
	unsigned int old;
	int count = spin_retry;

	while (1) {
		if (count-- <= 0) {
			smp_yield();
			count = spin_retry;
		}
		if (!arch_read_can_lock(rw))
			continue;
		old = rw->lock & 0x7fffffffU;
		if (_raw_compare_and_swap(&rw->lock, old, old + 1))
			return;
	}
}
EXPORT_SYMBOL(_raw_read_lock_wait);

void _raw_read_lock_wait_flags(arch_rwlock_t *rw, unsigned long flags)
{
	unsigned int old;
	int count = spin_retry;

	local_irq_restore(flags);
	while (1) {
		if (count-- <= 0) {
			smp_yield();
			count = spin_retry;
		}
		if (!arch_read_can_lock(rw))
			continue;
		old = rw->lock & 0x7fffffffU;
		local_irq_disable();
		if (_raw_compare_and_swap(&rw->lock, old, old + 1))
			return;
	}
}
EXPORT_SYMBOL(_raw_read_lock_wait_flags);

int _raw_read_trylock_retry(arch_rwlock_t *rw)
{
	unsigned int old;
	int count = spin_retry;

	while (count-- > 0) {
		if (!arch_read_can_lock(rw))
			continue;
		old = rw->lock & 0x7fffffffU;
		if (_raw_compare_and_swap(&rw->lock, old, old + 1))
			return 1;
	}
	return 0;
}
EXPORT_SYMBOL(_raw_read_trylock_retry);

void _raw_write_lock_wait(arch_rwlock_t *rw)
{
	int count = spin_retry;

	while (1) {
		if (count-- <= 0) {
			smp_yield();
			count = spin_retry;
		}
		if (!arch_write_can_lock(rw))
			continue;
		if (_raw_compare_and_swap(&rw->lock, 0, 0x80000000))
			return;
	}
}
EXPORT_SYMBOL(_raw_write_lock_wait);

void _raw_write_lock_wait_flags(arch_rwlock_t *rw, unsigned long flags)
{
	int count = spin_retry;

	local_irq_restore(flags);
	while (1) {
		if (count-- <= 0) {
			smp_yield();
			count = spin_retry;
		}
		if (!arch_write_can_lock(rw))
			continue;
		local_irq_disable();
		if (_raw_compare_and_swap(&rw->lock, 0, 0x80000000))
			return;
	}
}
EXPORT_SYMBOL(_raw_write_lock_wait_flags);

int _raw_write_trylock_retry(arch_rwlock_t *rw)
{
	int count = spin_retry;

	while (count-- > 0) {
		if (!arch_write_can_lock(rw))
			continue;
		if (_raw_compare_and_swap(&rw->lock, 0, 0x80000000))
			return 1;
	}
	return 0;
}
EXPORT_SYMBOL(_raw_write_trylock_retry);
