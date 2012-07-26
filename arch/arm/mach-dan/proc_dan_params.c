/*
	All files except if stated otherwise in the begining of the file are under the ISC license:
	-----------------------------------------------------------------------------------

	Copyright (c) 2010-2012 Design Art Networks Ltd.

	Permission to use, copy, modify, and/or distribute this software for any
	purpose with or without fee is hereby granted, provided that the above
	copyright notice and this permission notice appear in all copies.

	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/


/*
	All files except if stated otherwise in the begining of the file are under the ISC license:
	-----------------------------------------------------------------------------------

	Copyright (c) 2010-2012 Design Art Networks Ltd.

	Permission to use, copy, modify, and/or distribute this software for any
	purpose with or without fee is hereby granted, provided that the above
	copyright notice and this permission notice appear in all copies.

	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/


#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>

static char* proc_dan_params_addr = NULL;
static int proc_dan_params_size = -1;

static int proc_dan_params_show(struct seq_file *m, void *v)
{
	seq_write(m, proc_dan_params_addr, proc_dan_params_size);
	return 0;
}

static int proc_dan_params_open(struct inode *inode, struct file *file)
{
	if ( !proc_dan_params_addr )
	{
		u32 params_phys_addr = 0;
		char* umon_params = strstr(saved_command_line, "umon_params=");
		if ( umon_params )
			sscanf(umon_params, "umon_params=0x%x,%u", &params_phys_addr, &proc_dan_params_size);
		if ( params_phys_addr && (proc_dan_params_size > 0) )
			proc_dan_params_addr = ioremap(params_phys_addr, proc_dan_params_size);
	}
	if ( !proc_dan_params_addr )
		return -EPERM;

	return single_open(file, proc_dan_params_show, NULL);
}

static const struct file_operations proc_dan_params_fops = {
	.open    = proc_dan_params_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};


static const char PROC_DAN_DIR_NAME[] = "dan";
static const char PROC_DAN_PARAMS_FILE_NAME[] = "params";
static struct proc_dir_entry* proc_dan_dir = NULL;
static struct proc_dir_entry* proc_dan_params = NULL;

static int __init proc_dan_params_init(void)
{
	proc_dan_dir = proc_mkdir(PROC_DAN_DIR_NAME, NULL);
	if ( proc_dan_dir )
		proc_dan_params = proc_create(PROC_DAN_PARAMS_FILE_NAME, 0444, proc_dan_dir, &proc_dan_params_fops);

	return 0;
}
module_init(proc_dan_params_init);

static void proc_dan_params_exit(void)
{
	if ( proc_dan_dir )
		remove_proc_entry(PROC_DAN_PARAMS_FILE_NAME, proc_dan_dir);
	remove_proc_entry(PROC_DAN_DIR_NAME, NULL);
}
module_exit(proc_dan_params_exit);

