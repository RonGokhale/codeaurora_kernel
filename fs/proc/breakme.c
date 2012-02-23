/*
 *  breakme.c - crash-triggering test driver
 *
 *  Copyright (C) 2012 The Chromium OS Authors
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/cpumask.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/irqnr.h>
#include <asm/cputime.h>
#include <asm/uaccess.h>
#include <linux/chromeos_platform.h>

DEFINE_SPINLOCK(breakme_lock_me_up);

/* Null pointer dereference */
static void breakme_do_nullptr(void)
{
	*(unsigned long *)0 = 0;
}

/* BUG() */
static void breakme_do_bug(void)
{
	BUG();
}

/* hung_task stuck in unkillable D state */
static void breakme_do_hungtask(void)
{
	schedule_timeout_uninterruptible(MAX_SCHEDULE_TIMEOUT);
}

/* Panic */
static void breakme_do_panic(void)
{
	panic("Testing panic");
}

/* Set up a deadlock (call this twice) */
static void breakme_do_deadlock(void)
{
	spin_lock(&breakme_lock_me_up);
}

/* lockup */
static void breakme_do_softlockup(void)
{
	while (1)
		;
}

/* lockup with interrupts enabled */
static void breakme_do_irqlockup(void)
{
	spin_lock(&breakme_lock_me_up);
	while (1)
		;
}

/* lockup with interrupts disabled */
static void breakme_do_nmilockup(void)
{
	spin_lock_irq(&breakme_lock_me_up);
	while (1)
		;
}

/* set needs-recovery bit */
static void breakme_do_need_recovery(void)
{
	chromeos_set_need_recovery();
}

#define BREAKME_CMD(name) { #name, breakme_do_ ## name }
static struct breakme_cmd {
	char *name;
	void (*handler)(void);
} breakme_cmds[] = {
	BREAKME_CMD(nullptr),
	BREAKME_CMD(bug),
	BREAKME_CMD(hungtask),
	BREAKME_CMD(panic),
	BREAKME_CMD(deadlock),
	BREAKME_CMD(softlockup),
	BREAKME_CMD(irqlockup),
	BREAKME_CMD(nmilockup),
	BREAKME_CMD(need_recovery),
	{ }
};

#define MAX_BREAKME_WRITE 64
static ssize_t breakme_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	char kbuf[MAX_BREAKME_WRITE + 1];
	struct breakme_cmd *cmd;

	if (!count)
		return 0;

	if (count > MAX_BREAKME_WRITE)
		return -EINVAL;
	if (copy_from_user(&kbuf, buf, count))
		return -EFAULT;
	kbuf[min(count, sizeof(kbuf))-1] = '\0';

	for (cmd = breakme_cmds; cmd->handler; ++cmd) {
		if (!strcmp(kbuf, cmd->name)) {
			cmd->handler();
			break;
		}
	}

	return count;
}

static int breakme_show(struct seq_file *s, void *v)
{
	struct breakme_cmd *cmd;

	seq_printf(s, "Available commands:\n");
	for (cmd = breakme_cmds; cmd->name; ++cmd)
		seq_printf(s, "\t%s\n", cmd->name);
	return 0;
}

static int breakme_open(struct inode *inode, struct file *file)
{
	return single_open(file, breakme_show, NULL);
}

static struct file_operations proc_breakme_operations = {
	.open		= breakme_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= breakme_write,
	.release	= single_release,
};

void __init proc_breakme_init(void)
{
	proc_create("breakme", S_IWUSR, NULL, &proc_breakme_operations);
}
