#ifndef _X86_KERNEL_PROCESS_IO_H
#define _X86_KERNEL_PROCESS_IO_H

static inline void clear_thread_io_bitmap(struct task_struct *p)
{
#ifdef CONFIG_X86_IOPORT
	p->thread.io_bitmap_ptr = NULL;
#endif /* CONFIG_X86_IOPORT */
}

static inline int copy_io_bitmap(struct task_struct *me,
				 struct task_struct *p)
{
#ifdef CONFIG_X86_IOPORT
	if (unlikely(test_tsk_thread_flag(me, TIF_IO_BITMAP))) {
		p->thread.io_bitmap_ptr = kmemdup(me->thread.io_bitmap_ptr,
						  IO_BITMAP_BYTES, GFP_KERNEL);
		if (!p->thread.io_bitmap_ptr) {
			p->thread.io_bitmap_max = 0;
			return -ENOMEM;
		}
		set_tsk_thread_flag(p, TIF_IO_BITMAP);
	} else {
		p->thread.io_bitmap_ptr = NULL;
	}
#endif /* CONFIG_X86_IOPORT */

	return 0;
}

static inline void exit_thread_io(struct task_struct *me)
{
#ifdef CONFIG_X86_IOPORT
        struct thread_struct *t = &me->thread;
        unsigned long *bp = t->io_bitmap_ptr;

        if (bp) {
                struct tss_struct *tss = &per_cpu(init_tss, get_cpu());

                t->io_bitmap_ptr = NULL;
                clear_thread_flag(TIF_IO_BITMAP);
                /*
                 * Careful, clear this in the TSS too:
                 */
                memset(tss->io_bitmap, 0xff, t->io_bitmap_max);
                t->io_bitmap_max = 0;
                put_cpu();
                kfree(bp);
        }
#endif /* CONFIG_X86_IOPORT */
}

static inline void switch_iopl_mask(struct thread_struct *prev,
				    struct thread_struct *next)
{
#ifdef CONFIG_X86_IOPORT
        /*
         * Restore IOPL if needed.  In normal use, the flags restore
         * in the switch assembly will handle this.  But if the kernel
         * is running virtualized at a non-zero CPL, the popf will
         * not restore flags, so it must be done in a separate step.
         */
        if (get_kernel_rpl() && unlikely(prev->iopl != next->iopl))
                set_iopl_mask(next->iopl);
#endif /* CONFIG_X86_IOPORT */
}

static inline void switch_io_bitmap(struct tss_struct *tss,
				    struct task_struct *prev_p,
				    struct task_struct *next_p)
{
#ifdef CONFIG_X86_IOPORT
        struct thread_struct *prev, *next;
        prev = &prev_p->thread;
        next = &next_p->thread;

        if (test_tsk_thread_flag(next_p, TIF_IO_BITMAP)) {
                /*
                 * Copy the relevant range of the IO bitmap.
                 * Normally this is 128 bytes or less:
                 */
                memcpy(tss->io_bitmap, next->io_bitmap_ptr,
                       max(prev->io_bitmap_max, next->io_bitmap_max));
        } else if (test_tsk_thread_flag(prev_p, TIF_IO_BITMAP)) {
                /*
                 * Clear any possible leftover bits:
                 */
                memset(tss->io_bitmap, 0xff, prev->io_bitmap_max);
        }
#endif /* CONFIG_X86_IOPORT */
}

#endif /* _X86_KERNEL_PROCESS_IO_H */
