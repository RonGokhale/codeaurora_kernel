/*
 * Copyright 2010 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 *
 * Support the cycle counter clocksource and tile timer clock event device.
 */

#include <linux/time.h>
#include <linux/timex.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/hardirq.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/stop_machine.h>
#include <linux/timekeeper_internal.h>
#include <asm/irq_regs.h>
#include <asm/traps.h>
#include <asm/vdso.h>
#include <hv/hypervisor.h>
#include <arch/interrupts.h>
#include <arch/spr_def.h>


/*
 * Define the cycle counter clock source.
 */

/* How many cycles per second we are running at. */
static cycles_t cycles_per_sec;

cycles_t get_clock_rate(void)
{
	return cycles_per_sec;
}

#if CHIP_HAS_SPLIT_CYCLE()
cycles_t get_cycles(void)
{
	unsigned int high = __insn_mfspr(SPR_CYCLE_HIGH);
	unsigned int low = __insn_mfspr(SPR_CYCLE_LOW);
	unsigned int high2 = __insn_mfspr(SPR_CYCLE_HIGH);

	while (unlikely(high != high2)) {
		low = __insn_mfspr(SPR_CYCLE_LOW);
		high = high2;
		high2 = __insn_mfspr(SPR_CYCLE_HIGH);
	}

	return (((cycles_t)high) << 32) | low;
}
EXPORT_SYMBOL(get_cycles);
#endif

/*
 * We use a relatively small shift value so that sched_clock()
 * won't wrap around very often.
 */
#define SCHED_CLOCK_SHIFT 10

static unsigned long sched_clock_mult;
static long long sched_clock_offset;

static cycles_t clocksource_get_cycles(struct clocksource *cs)
{
	return get_cycles();
}

static struct clocksource cycle_counter_cs = {
	.name = "cycle counter",
	.rating = 300,
	.read = clocksource_get_cycles,
	.mask = CLOCKSOURCE_MASK(64),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

/*
 * Called very early from setup_arch() to set cycles_per_sec.
 * We initialize it early so we can use it to set up loops_per_jiffy.
 */
void __init setup_clock(void)
{
	cycles_per_sec = hv_sysconf(HV_SYSCONF_CPU_SPEED);
	sched_clock_mult =
		clocksource_hz2mult(cycles_per_sec, SCHED_CLOCK_SHIFT);
	sched_clock_offset = 0;
}

void __init calibrate_delay(void)
{
	loops_per_jiffy = get_clock_rate() / HZ;
	pr_info("Clock rate yields %lu.%02lu BogoMIPS (lpj=%lu)\n",
		loops_per_jiffy/(500000/HZ),
		(loops_per_jiffy/(5000/HZ)) % 100, loops_per_jiffy);
}

/* Called fairly late in init/main.c, but before we go smp. */
void __init time_init(void)
{
	/* Initialize and register the clock source. */
	clocksource_register_hz(&cycle_counter_cs, cycles_per_sec);

	/* Start up the tile-timer interrupt source on the boot cpu. */
	setup_tile_timer();
}

/*
 * Define the tile timer clock event device.  The timer is driven by
 * the TILE_[AUX_]TIMER_CONTROL register, which consists of a 31-bit down
 * counter, plus bit 31, which signifies that the counter has wrapped
 * from zero to (2**31) - 1.  The INT_[AUX_]TILE_TIMER interrupt will be
 * raised as long as bit 31 is set.
 */

#define MAX_TICK 0x7fffffff   /* we have 31 bits of countdown timer */

static int tile_timer_set_next_event(unsigned long ticks,
				     struct clock_event_device *evt)
{
	BUG_ON(ticks > MAX_TICK);
	__insn_mtspr(SPR_LINUX_TIMER_CONTROL, ticks);
	arch_local_irq_unmask_now(INT_LINUX_TIMER);
	return 0;
}

/*
 * Whenever anyone tries to change modes, we just mask interrupts
 * and wait for the next event to get set.
 */
static void tile_timer_set_mode(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
	arch_local_irq_mask_now(INT_LINUX_TIMER);
}

static DEFINE_PER_CPU(struct clock_event_device, tile_timer) = {
	.name = "tile timer",
	.features = CLOCK_EVT_FEAT_ONESHOT,
	.rating = 100,
	.irq = -1,
	.set_next_event = tile_timer_set_next_event,
	.set_mode = tile_timer_set_mode,
};

void setup_tile_timer(void)
{
	struct clock_event_device *evt = &__get_cpu_var(tile_timer);

	/* Mark as being for this cpu only. */
	evt->cpumask = cpumask_of(smp_processor_id());

	/* Start out with timer not firing. */
	arch_local_irq_mask_now(INT_LINUX_TIMER);

	/*
	 * Register tile timer.  Set min_delta to 1 microsecond, since
	 * it takes about that long to fire the interrupt.
	 */
	clockevents_config_and_register(evt, cycles_per_sec,
					cycles_per_sec / 1000000, MAX_TICK);
}

/* Called from the interrupt vector. */
void do_timer_interrupt(struct pt_regs *regs, int fault_num)
{
	struct pt_regs *old_regs = set_irq_regs(regs);
	struct clock_event_device *evt = &__get_cpu_var(tile_timer);

	/*
	 * Mask the timer interrupt here, since we are a oneshot timer
	 * and there are now by definition no events pending.
	 */
	arch_local_irq_mask(INT_LINUX_TIMER);

	/* Track time spent here in an interrupt context */
	irq_enter();

	/* Track interrupt count. */
	__get_cpu_var(irq_stat).irq_timer_count++;

	/* Call the generic timer handler */
	evt->event_handler(evt);

	/*
	 * Track time spent against the current process again and
	 * process any softirqs if they are waiting.
	 */
	irq_exit();

	set_irq_regs(old_regs);
}

/*
 * Scheduler clock - returns current time in nanosec units.
 * Note that with LOCKDEP, this is called during lockdep_init(), and
 * we will claim that sched_clock() is zero for a little while, until
 * we run setup_clock(), above.
 */
unsigned long long sched_clock(void)
{
	return clocksource_cyc2ns(get_cycles(), sched_clock_mult,
				  SCHED_CLOCK_SHIFT) + sched_clock_offset;
}

int setup_profiling_timer(unsigned int multiplier)
{
	return -EINVAL;
}

/*
 * Use the tile timer to convert nsecs to core clock cycles, relying
 * on it having the same frequency as SPR_CYCLE.
 */
cycles_t ns2cycles(unsigned long nsecs)
{
	/*
	 * We do not have to disable preemption here as each core has the same
	 * clock frequency.
	 */
	struct clock_event_device *dev = &__raw_get_cpu_var(tile_timer);
	return ((u64)nsecs * dev->mult) >> dev->shift;
}

void update_vsyscall_tz(void)
{
	/* Userspace gettimeofday will spin while this value is odd. */
	++vdso_data->tz_update_count;
	smp_wmb();
	vdso_data->tz_minuteswest = sys_tz.tz_minuteswest;
	vdso_data->tz_dsttime = sys_tz.tz_dsttime;
	smp_wmb();
	++vdso_data->tz_update_count;
}

void update_vsyscall(struct timekeeper *tk)
{
	struct timespec wall_time = tk_xtime(tk);
	struct timespec *wtm = &tk->wall_to_monotonic;
	struct clocksource *clock = tk->clock;

	if (clock != &cycle_counter_cs)
		return;

	/* Userspace gettimeofday will spin while this value is odd. */
	++vdso_data->tb_update_count;
	smp_wmb();
	vdso_data->xtime_tod_stamp = clock->cycle_last;
	vdso_data->xtime_clock_sec = wall_time.tv_sec;
	vdso_data->xtime_clock_nsec = wall_time.tv_nsec;
	vdso_data->wtom_clock_sec = wtm->tv_sec;
	vdso_data->wtom_clock_nsec = wtm->tv_nsec;
	vdso_data->mult = clock->mult;
	vdso_data->shift = clock->shift;
	smp_wmb();
	++vdso_data->tb_update_count;
}


#ifdef __tilegx__

/* Arguments to the _set_clock_rate stop_machine() handler. */
struct _set_clock_rate_args {
	unsigned int new_rate;
	int master_cpu;
};

/*
 * Flag used to tell other CPUs to proceed once the master CPU has changed
 * the actual CPU clock rate (barrier is positive), or failed to do so
 * (barrier is negative).
 */
static int _set_clock_rate_barrier;

/* Routine to actually do the clock rate change, called via stop_machine(). */
static int _set_clock_rate(void *arg)
{
	struct _set_clock_rate_args *args = arg;
	struct clock_event_device *evt = &__get_cpu_var(tile_timer);

	/*
	 * Only one CPU needs to change the timekeeping parameters and
	 * change the clock rate.
	 */
	if (args->master_cpu == smp_processor_id()) {
		unsigned long long old_sched_clock;
		long new_speed;
		cycle_t start_cycle;
		HV_SetSpeed hvss;

		/*
		 * Sync up the time before changing the clock rate.  If we
		 * aren't using the clocksource we think we are, bail.
		 */
		if (timekeeping_chfreq_prep(&cycle_counter_cs, &start_cycle)) {
			smp_wmb();
			_set_clock_rate_barrier = -1;
			return -ESRCH;
		}

		/*
		 * We'll adjust the offset below so that the new scheduler
		 * clock matches the value we read here, plus the time
		 * spent updating the CPU speed.  This causes us to lose a
		 * tiny bit of time, but it stays monotonic.
		 */
		old_sched_clock = sched_clock();

		/* Change the speed.  If that fails, bail. */
		hvss = hv_set_speed(args->new_rate, start_cycle, 0);
		new_speed = hvss.new_speed;
		if (new_speed < 0) {
			smp_wmb();
			_set_clock_rate_barrier = -1;
			return -ENOSYS;
		}

		/*
		 * Change the clocksource frequency, and update the
		 * timekeeping state to account for the time we spent
		 * changing the speed, then update our internal state.
		 */
		timekeeping_chfreq(new_speed, hvss.end_cycle, hvss.delta_ns);

		cycles_per_sec = new_speed;

		sched_clock_mult =
			clocksource_hz2mult(cycles_per_sec,
					    SCHED_CLOCK_SHIFT);

		sched_clock_offset = old_sched_clock -
				     (sched_clock() - sched_clock_offset) +
				     hvss.delta_ns;

		loops_per_jiffy = cycles_per_sec / HZ;

		smp_wmb();
		_set_clock_rate_barrier = 1;
	} else {
		/* Wait until the master CPU changes the speed, or fails. */
		while (!_set_clock_rate_barrier)
			udelay(10);
	}

	/*
	 * All CPUs need to change the event timer configuration, but we
	 * don't want to do anything if the master CPU failed to
	 * reconfigure the clocksource and change the speed.
	 */
	if (_set_clock_rate_barrier < 0)
		return 0;

	if (clockevents_update_freq(evt, cycles_per_sec) == -ETIME) {
		/*
		 * The event that we'd previously been set for is
		 * in the past.  Instead of just losing it, which
		 * causes havoc, make it happen right now.
		 */
		tile_timer_set_next_event(0, evt);
	}

	return 0;
}

/*
 * Change the clock speed, and return the speed we ended up with; both are
 * in hertz.
 */
unsigned int set_clock_rate(unsigned int new_rate)
{
	int stop_status;
	struct _set_clock_rate_args args = {
		.new_rate = new_rate,
		/*
		 * We just need a valid CPU here; we don't care if we get
		 * rescheduled somewhere else after we set this.
		 */
		.master_cpu = raw_smp_processor_id(),
	};

	_set_clock_rate_barrier = 0;
	smp_wmb();

	stop_status = stop_machine(_set_clock_rate, &args,
				   cpu_online_mask);

	if (stop_status)
		pr_err("Got unexpected status %d from stop_machine when "
		       "changing clock speed\n", stop_status);

	return cycles_per_sec;
};

#endif /* __tilegx__ */
