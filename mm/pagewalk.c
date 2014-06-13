#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/sched.h>
#include <linux/hugetlb.h>

static int get_reset_walk_control(struct mm_walk *walk)
{
	int ret = walk->control;
	/* Reset to default value */
	walk->control = PTWALK_NEXT;
	return ret;
}

static int walk_pte_range(pmd_t *pmd, unsigned long addr,
				unsigned long end, struct mm_walk *walk)
{
	struct mm_struct *mm = walk->mm;
	pte_t *pte;
	pte_t *orig_pte;
	int err = 0;

	walk->pmd = pmd;
	orig_pte = pte = pte_offset_map_lock(mm, pmd, addr, &walk->ptl);
	do {
		if (pte_none(*pte)) {
			if (walk->pte_hole)
				err = walk->pte_hole(addr, addr + PAGE_SIZE,
							walk);
			if (err)
				break;
			continue;
		}
		/*
		 * Callers should have their own way to handle swap entries
		 * in walk->pte_entry().
		 */
		err = walk->pte_entry(pte, addr, addr + PAGE_SIZE, walk);
		if (err)
		       break;
		switch (get_reset_walk_control(walk)) {
		case PTWALK_NEXT:
			continue;
		case PTWALK_DOWN:
			break;
		case PTWALK_BREAK:
			goto out_unlock;
		default:
			BUG();
		}
	} while (pte++, addr += PAGE_SIZE, addr < end);
out_unlock:
	pte_unmap_unlock(orig_pte, walk->ptl);
	cond_resched();
	return addr == end ? 0 : err;
}

static int walk_pmd_range(pud_t *pud, unsigned long addr,
				unsigned long end, struct mm_walk *walk)
{
	pmd_t *pmd;
	unsigned long next;
	int err = 0;
	spinlock_t *ptl;

	pmd = pmd_offset(pud, addr);
	do {
again:
		next = pmd_addr_end(addr, end);

		if (pmd_none(*pmd)) {
			if (walk->pte_hole)
				err = walk->pte_hole(addr, next, walk);
			if (err)
				break;
			continue;
		}

		/*
		 * We don't take compound_lock() here but no race with splitting
		 * thp happens because:
		 *  - if pmd_trans_huge_lock() returns 1, the relevant thp is
		 *    not under splitting, which means there's no concurrent
		 *    thp split,
		 *  - if another thread runs into split_huge_page() just after
		 *    we entered this if-block, the thread must wait for page
		 *    table lock to be unlocked in __split_huge_page_splitting(),
		 *    where the main part of thp split is not executed yet.
		 */
		if (walk->pmd_entry) {
			if (pmd_trans_huge_lock(pmd, walk->vma, &ptl) == 1) {
				err = walk->pmd_entry(pmd, addr, next, walk);
				spin_unlock(ptl);
			}
			if (err)
				break;
			switch (get_reset_walk_control(walk)) {
			case PTWALK_NEXT:
				continue;
			case PTWALK_DOWN:
				break;
			default:
				BUG();
			}
		}

		if (walk->pte_entry) {
			if (walk->vma) {
				split_huge_page_pmd(walk->vma, addr, pmd);
				if (pmd_trans_unstable(pmd))
					goto again;
			}
			err = walk_pte_range(pmd, addr, next, walk);
			if (err)
				break;
		}
	} while (pmd++, addr = next, addr < end);

	return err;
}

static int walk_pud_range(pgd_t *pgd, unsigned long addr,
				unsigned long end, struct mm_walk *walk)
{
	pud_t *pud;
	unsigned long next;
	int err = 0;

	pud = pud_offset(pgd, addr);
	do {
		next = pud_addr_end(addr, end);

		if (pud_none_or_clear_bad(pud)) {
			if (walk->pte_hole)
				err = walk->pte_hole(addr, next, walk);
			if (err)
				break;
			continue;
		}

		if (walk->pmd_entry || walk->pte_entry) {
			err = walk_pmd_range(pud, addr, next, walk);
			if (err)
				break;
		}
	} while (pud++, addr = next, addr < end);

	return err;
}

static int walk_pgd_range(unsigned long addr, unsigned long end,
			struct mm_walk *walk)
{
	pgd_t *pgd;
	unsigned long next;
	int err = 0;

	pgd = pgd_offset(walk->mm, addr);
	do {
		next = pgd_addr_end(addr, end);

		if (pgd_none_or_clear_bad(pgd)) {
			if (walk->pte_hole)
				err = walk->pte_hole(addr, next, walk);
			if (err)
				break;
			continue;
		}

		if (walk->pmd_entry || walk->pte_entry) {
			err = walk_pud_range(pgd, addr, next, walk);
			if (err)
				break;
		}
	} while (pgd++, addr = next, addr < end);

	return err;
}

#ifdef CONFIG_HUGETLB_PAGE
static unsigned long hugetlb_entry_end(struct hstate *h, unsigned long addr,
				       unsigned long end)
{
	unsigned long boundary = (addr & huge_page_mask(h)) + huge_page_size(h);
	return boundary < end ? boundary : end;
}

static int walk_hugetlb_range(unsigned long addr, unsigned long end,
				struct mm_walk *walk)
{
	struct mm_struct *mm = walk->mm;
	struct vm_area_struct *vma = walk->vma;
	struct hstate *h = hstate_vma(vma);
	unsigned long next;
	unsigned long hmask = huge_page_mask(h);
	pte_t *pte;
	int err = 0;
	spinlock_t *ptl;

	do {
		next = hugetlb_entry_end(h, addr, end);
		pte = huge_pte_offset(walk->mm, addr & hmask);
		if (!pte)
			continue;
		ptl = huge_pte_lock(h, mm, pte);
		/*
		 * Callers should have their own way to handle swap entries
		 * in walk->hugetlb_entry().
		 */
		if (walk->hugetlb_entry)
			err = walk->hugetlb_entry(pte, addr, next, walk);
		spin_unlock(ptl);
		if (err)
			break;
	} while (addr = next, addr != end);
	cond_resched();
	return err;
}

#else /* CONFIG_HUGETLB_PAGE */
static inline int walk_hugetlb_range(unsigned long addr, unsigned long end,
				struct mm_walk *walk)
{
	return 0;
}

#endif /* CONFIG_HUGETLB_PAGE */

/*
 * Decide whether we really walk over the current vma on [@start, @end) or
 * skip it. If we walk over it, we should set @walk->control to PTWALK_DOWN.
 * Otherwise, we skip it. The return value is used to control the current
 * walking to continue (for zero) or terminate (for non-zero).
 *
 * We fall through to the default check if the caller doesn't define its own
 * test_walk() callback.
 */
static int walk_page_test(unsigned long start, unsigned long end,
			struct mm_walk *walk)
{
	struct vm_area_struct *vma = walk->vma;

	if (walk->test_walk)
		return walk->test_walk(start, end, walk);

	/*
	 * Do not walk over vma(VM_PFNMAP), because we have no valid struct
	 * page backing a VM_PFNMAP range. See also commit a9ff785e4437.
	 */
	if (!(vma->vm_flags & VM_PFNMAP))
		walk->control = PTWALK_DOWN;
	return 0;
}

static int __walk_page_range(unsigned long start, unsigned long end,
			struct mm_walk *walk)
{
	int err = 0;
	struct vm_area_struct *vma = walk->vma;

	if (vma && is_vm_hugetlb_page(vma)) {
		if (walk->hugetlb_entry)
			err = walk_hugetlb_range(start, end, walk);
	} else
		err = walk_pgd_range(start, end, walk);

	return err;
}

/**
 * walk_page_range - walk page table with caller specific callbacks
 *
 * Recursively walk the page table tree of the process represented by
 * @walk->mm within the virtual address range [@start, @end). In walking,
 * we can call caller-specific callback functions against each entry.
 *
 * Before starting to walk page table, some callers want to check whether
 * they really want to walk over the vma (for example by checking vm_flags.)
 * walk_page_test() and @walk->test_walk() do that check.
 *
 * If any callback returns a non-zero value, the page table walk is aborted
 * immediately and the return value is propagated back to the caller.
 * Note that the meaning of the positive returned value can be defined
 * by the caller for its own purpose.
 *
 * If the caller defines multiple callbacks in different levels, the
 * callbacks are called in depth-first manner. It could happen that
 * multiple callbacks are called on a address. For example if some caller
 * defines test_walk(), pmd_entry(), and pte_entry(), then callbacks are
 * called in the order of test_walk(), pmd_entry(), and pte_entry().
 * If you don't want to go down to lower level at some point and move to
 * the next entry in the same level, you set @walk->control to PTWALK_DOWN.
 * For example if you succeed to handle some pmd entry as trans_huge entry,
 * you need not call walk_pte_range() any more, so set it to avoid that.
 * We can't determine whether to go down to lower level with the return
 * value of the callback, because the whole range of return values (0, >0,
 * and <0) are used up for other meanings.
 *
 * Each callback can access to the vma over which it is doing page table
 * walk right now via @walk->vma. @walk->vma is set to NULL in walking
 * outside a vma. If you want to access to some caller-specific data from
 * callbacks, @walk->private should be helpful.
 *
 * Locking:
 *   Callers of walk_page_range() and walk_page_vma() should hold
 *   @walk->mm->mmap_sem, because these function traverse vma list and/or
 *   access to vma's data. And page table lock is held during running
 *   pmd_entry() and pte_entry().
 */
int walk_page_range(unsigned long start, unsigned long end,
		    struct mm_walk *walk)
{
	int err = 0;
	struct vm_area_struct *vma;
	unsigned long next;

	if (start >= end)
		return -EINVAL;

	if (!walk->mm)
		return -EINVAL;

	VM_BUG_ON(!rwsem_is_locked(&walk->mm->mmap_sem));

	do {
		vma = find_vma(walk->mm, start);
		if (!vma) { /* after the last vma */
			walk->vma = NULL;
			next = end;
		} else if (start < vma->vm_start) { /* outside the found vma */
			walk->vma = NULL;
			next = vma->vm_start;
		} else { /* inside the found vma */
			walk->vma = vma;
			next = min(end, vma->vm_end);

			err = walk_page_test(start, next, walk);
			if (err)
				break;
			switch (get_reset_walk_control(walk)) {
			case PTWALK_NEXT:
				continue;
			case PTWALK_DOWN:
				break;
			default:
				BUG();
			}
		}
		err = __walk_page_range(start, next, walk);
		if (err)
			break;
	} while (start = next, start < end);
	return err;
}

int walk_page_vma(struct vm_area_struct *vma, struct mm_walk *walk)
{
	int err;

	if (!walk->mm)
		return -EINVAL;

	VM_BUG_ON(!rwsem_is_locked(&walk->mm->mmap_sem));
	VM_BUG_ON(!vma);
	walk->vma = vma;
	err = walk_page_test(vma->vm_start, vma->vm_end, walk);
	if (err)
		return err;
	switch (get_reset_walk_control(walk)) {
	case PTWALK_NEXT:
		return 0;
	case PTWALK_DOWN:
		break;
	default:
		BUG();
	}
	return __walk_page_range(vma->vm_start, vma->vm_end, walk);
}
