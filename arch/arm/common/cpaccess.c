/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/sysrq.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/kernel_stat.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/sysdev.h>
#include <asm/cpaccess.h>

/*
 * CP parameters
 */
static unsigned long cp_op1, cp_op2, cp_crn, cp_crm, cp_cp = 15;

static struct sysdev_class cpaccess_sysclass = {
	.name = "cpaccess",
};

/*
 * dp_cpregister_rw - Read/Write CP registers
 * @write:		1 for Write and 0 for Read operation
 * @write_value:	Write value for write and 0 for read
 *
 * Returns value read from CP register
 */
static unsigned long
do_cpregister_rw(int write, unsigned long write_value)
{
	unsigned long opcode, *p_opcode;

	/*
	 * Mask the crn, crm, op1, op2 and cp values so they do not
	 * interfer with other fields of the op code.
	 */
	cp_cp  &= 0xF;
	cp_crn &= 0xF;
	cp_crm &= 0xF;
	cp_op1 &= 0x7;
	cp_op2 &= 0x7;

	/*
	 * Base MRC opcode for MIDR is EE100010,
	 * MCR is 0xEE000010
	 */
	opcode = (write == 1 ? 0xEE000010 : 0xEE100010);
	opcode |= (cp_crn<<16) | (cp_crm<<0) | (cp_op1<<21) |
					(cp_op2<<5) | (cp_cp << 8);

	/*
	 *  Grab address of the current function that as an MRC as the first
	 *  instruction (it is in asm) and returns the value in R0.
	 *
	 *  Set the new op code, then flush the code.  We use "user range"
	 *  here because "kernel range" is assumed to be coherent and will
	 *  not contain modified code...User range performs both the dcache
	 *  clean and the icache invalidate.
	 */
	p_opcode = (unsigned long *)&do_asm_cp;
	*p_opcode = opcode;
	flush_cache_user_range(0, (unsigned long)p_opcode,
		(unsigned long)(p_opcode+sizeof(long)));
	/*
	 * Execute the function with the "modified" op code, format and
	 * return the result (it is in R0).
	 */
	return do_asm_cp(write_value);
}

/*
 * cp_register_write_sysfs - sysfs interface for writing to CP registers
 * @dev:	sys device
 * @attr:	device attribute
 * @buf:	write value
 * @cnt:	not used
 *
 */
static ssize_t
cp_register_write_sysfs(struct sys_device *dev, struct sysdev_attribute *attr,
						 const char *buf, size_t cnt)
{
	unsigned long write_value;

	sscanf(buf, "%lx", &write_value);
	return do_cpregister_rw(1, write_value);
}

/*
 * cp_register_read_sysfs - sysfs interface for reading CP registers
 * @dev:        sys device
 * @attr:       device attribute
 * @buf:        write value
 *
 * Code to read in the CPxx crn, crm, op1, op2 variables, or into
 * the base MRC opcode, store to executable memory, clean/invalidate
 * caches and then execute the new instruction and provide the
 * result to the caller.
 */
static ssize_t
cp_register_read_sysfs(struct sys_device *dev, struct sysdev_attribute *attr,
								char *buf)
{
	return sprintf(buf, "CP register value: %lx\n", do_cpregister_rw(0, 0));
}

/*
 * cp_write_params_sysfs - sysfs interface for reading CP registers
 * @dev:        sys device
 * @attr:       device attribute
 * @buf:        write value
 * @cnt:        not used
 *
 * Write the values for the op1, op2, crn, crm, cp values
 */
static ssize_t
cp_write_params_sysfs(struct sys_device *dev, struct sysdev_attribute *attr,
						const char *buf, size_t cnt)
{
	if (buf == NULL) {
		printk(KERN_ERR "sysfs write buffer is NULL\n");
		return cnt;
	}

	sscanf(buf, "%lu:%lu:%lu:%lu:%lu", &cp_cp, &cp_op1, &cp_crn , &cp_crm,
								&cp_op2);
	return cnt;
}

/*
 * cp_read_params_sysfs - sysfs interface for reading CP registers
 * @dev:        sys device
 * @attr:       device attribute
 * @buf:        write value
 *
 * Show the values of the op1, op2, CRn, CRm, cp variables.
 */
static ssize_t
cp_read_params_sysfs(struct sys_device *dev, struct sysdev_attribute *attr,
								char *buf)
{
	return sprintf(buf, "%lu:%lu:%lu:%lu:%lu\n", cp_cp, cp_op1, cp_crn,
							cp_crm, cp_op2);
}

/*
 * Setup sysfs files
 */
SYSDEV_ATTR(cp_parameters, 0644, cp_read_params_sysfs, cp_write_params_sysfs);

SYSDEV_ATTR(cp_read, 0644, cp_register_read_sysfs, cp_register_write_sysfs);

static struct sys_device device_cpaccess = {
	.id     = 0,
	.cls    = &cpaccess_sysclass,
};

/*
 * init_cpaccess_sysfs - initialize sys devices
 */
static int __init init_cpaccess_sysfs(void)
{
	int error = sysdev_class_register(&cpaccess_sysclass);

	if (!error)
		error = sysdev_register(&device_cpaccess);

	if (!error)
		error = sysdev_create_file(&device_cpaccess,
					&attr_cp_parameters);

	if (!error)
		error = sysdev_create_file(&device_cpaccess, &attr_cp_read);

	if (error)
		printk(KERN_ERR "Error initializing cpaccess module\n");

	return error;
}

static void __exit exit_cpaccess_sysfs(void)
{
	sysdev_remove_file(&device_cpaccess, &attr_cp_parameters);
	sysdev_remove_file(&device_cpaccess, &attr_cp_read);
	sysdev_unregister(&device_cpaccess);
	sysdev_class_unregister(&cpaccess_sysclass);
}

module_init(init_cpaccess_sysfs);
module_exit(exit_cpaccess_sysfs);
MODULE_LICENSE("Dual BSD/GPL");
