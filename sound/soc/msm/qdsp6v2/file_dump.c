
#define FILE_CAPTURE

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/msm_audio_qcp.h>
#include <linux/delay.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>

#ifdef FILE_CAPTURE
    struct file *f_write = NULL;
    struct file *f_read = NULL;
    mm_segment_t orig_fs;
    int length_write = 0;
    int length_read = 0;
    static char *fwrite = "/data/dump";
    static char *fread = "/data/filename";
#endif

int filedump_open(void)
{
#ifdef FILE_CAPTURE
    f_write = filp_open(fwrite, O_WRONLY | O_CREAT, 0600);

    if (!f_write || !f_write->f_op || !f_write->f_op->read) {
      printk("WARNING: File (read) object is a null pointer!!!\n");
      mdelay(100000);
      return -EINVAL;
    }
    printk("sucess in opening write file\n");
    f_write->f_pos = 0;

    f_read = filp_open(fread, O_WRONLY | O_CREAT, 0600);

    if (!f_read || !f_read->f_op || !f_read->f_op->read) {
      printk("WARNING: File (read) object is a null pointer!!!\n");
      mdelay(100000);
      return -EINVAL;
    }
    printk("sucess in opening read file\n");
    f_read->f_pos = 0;

    /* Use get_fs() and set_fs() to temporarily modify the addr_limit
    field of the current task_struct. This will allow the read to
    use a buffer whose address is not in use space.
    */
    orig_fs = get_fs();
    set_fs(KERNEL_DS);
#endif
    return 0;
}
//EXPORT_SYMBOL(filedump_open);

int filedump_write_open(const char* fwrite)
{
#ifdef FILE_CAPTURE
    f_write = filp_open(fwrite, O_WRONLY | O_CREAT, 0600);

    if (!f_write || !f_write->f_op || !f_write->f_op->read) {
      printk("WARNING: File (read) object is a null pointer!!!\n");
      mdelay(100000);
      return -EINVAL;
    }
    printk("sucess in opening write file\n");
    f_write->f_pos = 0;
    /* Use get_fs() and set_fs() to temporarily modify the addr_limit
    field of the current task_struct. This will allow the read to
    use a buffer whose address is not in use space.
    */
    orig_fs = get_fs();
    set_fs(KERNEL_DS);
#endif
    return 0;
}
EXPORT_SYMBOL(filedump_write_open);

int filedump_read_open(const char* fread)
{
#ifdef FILE_CAPTURE
    f_read = filp_open(fread, O_WRONLY | O_CREAT, 0600);

    if (!f_read || !f_read->f_op || !f_read->f_op->read) {
      printk("WARNING: File (read) object is a null pointer!!!\n");
      mdelay(100000);
      return -EINVAL;
    }
    printk("sucess in opening read file\n");
    f_read->f_pos = 0;

    /* Use get_fs() and set_fs() to temporarily modify the addr_limit
    field of the current task_struct. This will allow the read to
    use a buffer whose address is not in use space.
    */
    orig_fs = get_fs();
    set_fs(KERNEL_DS);
#endif
    return 0;
}
EXPORT_SYMBOL(filedump_read_open);

size_t filedump_write(const char *buf, size_t count)
{
#ifdef FILE_CAPTURE
	printk("write to file \n");
    length_write = f_write->f_op->write(f_write, buf, count, &f_write->f_pos);
    if ( length_write == 0 ) {
      printk("failed to write \n");
    }
    printk("length_write %d\n",length_write);
    return length_write;
#else
    return 0;
#endif
}
//EXPORT_SYMBOL(filedump_write);

size_t filedump_read(char *buf, size_t count)
{
#ifdef FILE_CAPTURE
    length_read = f_read->f_op->read(f_read, buf, count, &f_read->f_pos);
    if ( length_read == 0 ) {
      printk("failed to read \n");
    }
    printk("length_read %d\n",length_read);
    return length_read;
#else
    return 0;
#endif
}
EXPORT_SYMBOL(filedump_read);

int filedump_close(void)
{
#ifdef FILE_CAPTURE
	printk("filedump_close\n");
    if(f_write)
      filp_close(f_write, current->files);
    else if(f_read)
      filp_close(f_read, current->files);
#endif
    return 0;
}
//EXPORT_SYMBOL(filedump_close);

static int __init filedump_init(void)
{
    return 0;
}

device_initcall(filedump_init);
