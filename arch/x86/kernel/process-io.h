#ifndef _X86_KERNEL_PROCESS_IO_H
#define _X86_KERNEL_PROCESS_IO_H

static inline int copy_io_bitmap(struct task_struct *me,
				 struct task_struct *p)
{
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

	return 0;
}

#endif /* _X86_KERNEL_PROCESS_IO_H */
