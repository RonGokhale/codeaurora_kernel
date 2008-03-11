/* arch/arm/mach-msm/smd.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/arch/msm_smd.h>
#include <asm/arch/msm_iomap.h>
#include <asm/io.h>

#include "smd_private.h"
#include "proc_comm.h"

#define MODULE_NAME "msm_smd"

void *smem_find(unsigned id, unsigned size);

static unsigned last_heap_free = 0xffffffff;

#if 0
#define D(x...) printk(x)
#else
#define D(x...) do {} while (0)
#endif

#define MSM_A2M_INT(n) (MSM_CSR_BASE + 0x400 + (n) * 4)

static inline void notify_other_smsm(void)
{
	writel(1, MSM_A2M_INT(5));
}

static inline void notify_other_smd(void)
{
	writel(1, MSM_A2M_INT(0));
}

static inline void notify_other_proc_comm(void)
{
	writel(1, MSM_A2M_INT(6));
}

#define APP_COMMAND 0x00
#define APP_STATUS  0x04
#define APP_DATA1   0x08
#define APP_DATA2   0x0C

#define MDM_COMMAND 0x10
#define MDM_STATUS  0x14
#define MDM_DATA1   0x18
#define MDM_DATA2   0x1C

static DEFINE_SPINLOCK(proc_comm_lock);
 
int msm_proc_comm(unsigned cmd, unsigned *data1, unsigned *data2)
{
	unsigned long flags;
	unsigned base = MSM_SHARED_RAM_BASE;
	int ret = -EIO;

	spin_lock_irqsave(&proc_comm_lock, flags);

	while (readl(base + MDM_STATUS) != PCOM_READY) {
		/* XXX check for A9 reset */
	}

	writel(cmd, base + APP_COMMAND);
	if (data1)
		writel(*data1, base + APP_DATA1);
	if (data2)
		writel(*data2, base + APP_DATA2);

	notify_other_proc_comm();
	while (readl(base + APP_COMMAND) != PCOM_CMD_DONE) {
		/* XXX check for A9 reset */
	}

	if (readl(base + APP_STATUS) != PCOM_CMD_FAIL) {
		if (data1)
			*data1 = readl(base + APP_DATA1);
		if (data2)
			*data2 = readl(base + APP_DATA2);
		ret = 0;
	}

	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return ret;
}

void smd_diag(void)
{
	char *x;

	x = smem_find(ID_DIAG_ERR_MSG, SZ_DIAG_ERR_MSG);
	if (x != 0) {
		x[SZ_DIAG_ERR_MSG - 1] = 0;
		printk("smem: DIAG '%s'\n", x);
	}
}

#define SMD_SS_CLOSED            0x00000000
#define SMD_SS_OPENING           0x00000001
#define SMD_SS_OPENED            0x00000002
#define SMD_SS_FLUSHING          0x00000003
#define SMD_SS_CLOSING           0x00000004
#define SMD_SS_RESET             0x00000005
#define SMD_SS_RESET_OPENING     0x00000006

#define SMD_BUF_SIZE 8192
#define SMD_CHANNELS 64

#define SMD_HEADER_SIZE 20


/* the spinlock is used to synchronize between the
** irq handler and code that mutates the channel
** list or fiddles with channel state
*/
static DEFINE_SPINLOCK(smd_lock);
static DEFINE_SPINLOCK(smem_lock);

/* the mutex is used during open() and close()
** operations to avoid races while creating or
** destroying smd_channel structures
*/
static DEFINE_MUTEX(smd_creation_mutex);

static int smd_initialized = 0;

struct smd_half_channel
{
	unsigned state;
	unsigned char fDSR;
	unsigned char fCTS;
	unsigned char fCD;
	unsigned char fRI;
	unsigned char fHEAD;
	unsigned char fTAIL;
	unsigned char fSTATE;
	unsigned char fUNUSED;
	unsigned tail;
	unsigned head;
	unsigned char data[SMD_BUF_SIZE];
};

struct smd_shared
{
	struct smd_half_channel ch0;
	struct smd_half_channel ch1;
};

struct smd_channel
{
	volatile struct smd_half_channel *send;
	volatile struct smd_half_channel *recv;
	struct list_head ch_list;

	unsigned current_packet;
	unsigned n;
	void *priv;
	void (*notify)(void *priv, unsigned flags);

	int (*read)(smd_channel_t *ch, void *data, int len);
	int (*write)(smd_channel_t *ch, const void *data, int len);
	int (*read_avail)(smd_channel_t *ch);
	int (*write_avail)(smd_channel_t *ch);

	void (*update_state)(smd_channel_t *ch);
	unsigned last_state;
};

struct smem_pdev_entry
{
	int start_smem_id;
	char *name;
	int start_rel_id;
	int num_devices;
};

static LIST_HEAD(smd_ch_list);

static struct platform_device *p_devs[128];
static struct work_struct probe_work;

static struct smem_pdev_entry pdev_creation_table[] = {
	{ ID_SMD_CHANNELS,	"smd_channel",	0,	64 },
	{ -1, NULL, -1 }
};

static void smd_channel_probe_worker(struct work_struct *work)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	struct smem_heap_entry *toc = shared->heap_toc;
	struct smem_pdev_entry *v;

	for (v = pdev_creation_table; v->start_smem_id != -1; v++) {
		int i;

		for (i = v->start_smem_id; i < (v->start_smem_id + v->num_devices); i++) {
			if (toc[i].allocated && !p_devs[i]) {
				char name[20];
				int alloc_len;

				if (v->start_rel_id != -1)
					sprintf(name, "%s_%.2d", v->name,
						i - v->start_smem_id + v->start_rel_id);
				else
					strcpy(name, v->name);

				alloc_len = sizeof(struct platform_device) + strlen(name) + 1;
				p_devs[i] = kmalloc(alloc_len, GFP_KERNEL);
				if (!p_devs[i]) {
					printk(KERN_ERR "Out of resources\n");
					return;
				}

				memset(p_devs[i], 0, alloc_len);
				p_devs[i]->id = -1;
				p_devs[i]->name = ((void *) p_devs[i]) + sizeof(struct platform_device);

				strcpy(p_devs[i]->name, name);
				printk(KERN_INFO "%s: registering pdev '%s'\n", __FUNCTION__, name);
				platform_device_register(p_devs[i]);
			}
		}
	}
}

static char *chstate(unsigned n)
{
	switch (n) {
	case SMD_SS_CLOSED:        return "CLOSED";
	case SMD_SS_OPENING:       return "OPENING";
	case SMD_SS_OPENED:        return "OPENED";
	case SMD_SS_FLUSHING:      return "FLUSHING";
	case SMD_SS_CLOSING:       return "CLOSING";
	case SMD_SS_RESET:         return "RESET";
	case SMD_SS_RESET_OPENING: return "ROPENING";
	default:                   return "UNKNOWN";
	}
}

/* how many bytes are available for reading */
static int smd_stream_read_avail(struct smd_channel *ch)
{
	return (ch->recv->head - ch->recv->tail) & (SMD_BUF_SIZE - 1);
}

/* how many bytes we are free to write */
static int smd_stream_write_avail(struct smd_channel *ch)
{
	return (SMD_BUF_SIZE - 1) -
		((ch->send->head - ch->send->tail) & (SMD_BUF_SIZE - 1));
}

static int smd_packet_read_avail(struct smd_channel *ch)
{
	if (ch->current_packet) {
		int n = smd_stream_read_avail(ch);
		if (n > ch->current_packet)
			n = ch->current_packet;
		return n;
	} else {
		return 0;
	}
}

static int smd_packet_write_avail(struct smd_channel *ch)
{
	int n = smd_stream_write_avail(ch);
	return n > SMD_HEADER_SIZE ? n - SMD_HEADER_SIZE : 0;
}

static int ch_is_open(struct smd_channel *ch)
{
	return (ch->recv->state == SMD_SS_OPENED) &&
		(ch->send->state == SMD_SS_OPENED);
}

/* provide a pointer and length to readable data in the fifo */
static unsigned ch_read_buffer(struct smd_channel *ch, void **ptr)
{
	unsigned head = ch->recv->head;
	unsigned tail = ch->recv->tail;
	*ptr = (void *) (ch->recv->data + tail);

	if (tail <= head) {
		return head - tail;
	} else {
		return SMD_BUF_SIZE - tail;
	}
}

/* advance the fifo read pointer after data from ch_read_buffer is consumed */
static void ch_read_done(struct smd_channel *ch, unsigned count)
{
	BUG_ON(count > smd_stream_read_avail(ch));
	ch->recv->tail = (ch->recv->tail + count) & (SMD_BUF_SIZE - 1);
	ch->recv->fTAIL = 1;
}

/* basic read interface to ch_read_{buffer,done} used
** by smd_*_read() and update_packet_state()
** will read-and-discard if the _data pointer is null
*/
static int ch_read(struct smd_channel *ch, void *_data, int len)
{
	void *ptr;
	unsigned n;
	unsigned char *data = _data;
	int orig_len = len;

	while (len > 0) {
		n = ch_read_buffer(ch, &ptr);
		if (n == 0)
			break;

		if (n > len)
			n = len;
		if (_data)
			memcpy(data, ptr, n);

		data += n;
		len -= n;
		ch_read_done(ch, n);
	}

	return orig_len - len;
}

static void update_stream_state(struct smd_channel *ch)
{
	/* streams have no special state requiring updating */
}

static void update_packet_state(struct smd_channel *ch)
{
	unsigned hdr[5];
	int r;

	/* can't do anything if we're in the middle of a packet */
	if (ch->current_packet != 0) return;

	/* don't bother unless we can get the full header */
	if (smd_stream_read_avail(ch) < SMD_HEADER_SIZE) return;

	r = ch_read(ch, hdr, SMD_HEADER_SIZE);
	BUG_ON(r != SMD_HEADER_SIZE);

	ch->current_packet = hdr[0];
}

/* provide a pointer and length to next free space in the fifo */
static unsigned ch_write_buffer(struct smd_channel *ch, void **ptr)
{
	unsigned head = ch->send->head;
	unsigned tail = ch->send->tail;
	*ptr = (void *) (ch->send->data + head);

	if (head < tail) {
		return tail - head - 1;
	} else {
		if (tail == 0) {
			return SMD_BUF_SIZE - head - 1;
		} else {
			return SMD_BUF_SIZE - head;
		}
	}
}

/* advace the fifo write pointer after freespace from ch_write_buffer is filled */
static void ch_write_done(struct smd_channel *ch, unsigned count)
{
	BUG_ON(count > smd_stream_write_avail(ch));
	ch->send->head = (ch->send->head + count) & (SMD_BUF_SIZE - 1);
	ch->send->fHEAD = 1;
}

static void hc_set_state(volatile struct smd_half_channel *hc, unsigned n)
{
	if (n == SMD_SS_OPENED) {
		hc->fDSR = 1;
		hc->fCTS = 1;
		hc->fCD = 1;
	} else {
		hc->fDSR = 0;
		hc->fCTS = 0;
		hc->fCD = 0;
	}
	hc->state = n;
	hc->fSTATE = 1;
	notify_other_smd();
}

static void do_smd_probe(void)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	if (shared->heap_info.free_offset != last_heap_free) {
		last_heap_free = shared->heap_info.free_offset;
		schedule_work(&probe_work);
	}
}

static irqreturn_t smd_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct smd_channel *ch;
	int do_notify = 0;
	unsigned ch_flags;
	unsigned tmp;
/*	D("<SMD>\n"); */

	spin_lock_irqsave(&smd_lock, flags);
	list_for_each_entry(ch, &smd_ch_list, ch_list) {
		if (ch_is_open(ch)) {
			ch_flags = 0;
			if (ch->recv->fHEAD) {
				ch->recv->fHEAD = 0;
				ch_flags |= 1;
				do_notify |= 1;
			}
			if (ch->recv->fTAIL) {
				ch->recv->fTAIL = 0;
				ch_flags |= 2;
				do_notify |= 1;
			}
			if (ch->recv->fSTATE) {
				ch->recv->fSTATE = 0;
				ch_flags |= 4;
				do_notify |= 1;
			}
			tmp = ch->recv->state;
			if (tmp != ch->last_state) {
				ch->last_state = tmp;
				if (tmp == SMD_SS_OPENED) {
					ch->notify(ch->priv, SMD_EVENT_OPEN);
				} else {
					ch->notify(ch->priv, SMD_EVENT_CLOSE);
				}
			}
			if (ch_flags) {
				ch->update_state(ch);
				ch->notify(ch->priv, SMD_EVENT_DATA);
			}
		}
	}
	if (do_notify) notify_other_smd();
	spin_unlock_irqrestore(&smd_lock, flags);
	do_smd_probe();
	return IRQ_HANDLED;
}

void smd_kick(smd_channel_t *ch)
{
	unsigned long flags;
	unsigned tmp;

	spin_lock_irqsave(&smd_lock, flags);
	ch->update_state(ch);
	tmp = ch->recv->state;
	if (tmp != ch->last_state) {
		ch->last_state = tmp;
		if (tmp == SMD_SS_OPENED) {
			ch->notify(ch->priv, SMD_EVENT_OPEN);
		} else {
			ch->notify(ch->priv, SMD_EVENT_CLOSE);
		}
	}
	ch->notify(ch->priv, SMD_EVENT_DATA);
	notify_other_smd();
	spin_unlock_irqrestore(&smd_lock, flags);
}

static int smd_is_packet(int chn)
{
	if ((chn > 4) || (chn == 1)) {
		return 1;
	} else {
		return 0;
	}
}

static int smd_stream_write(smd_channel_t *ch, const void *_data, int len)
{
	void *ptr;
	const unsigned char *buf = _data;
	unsigned xfer;
	int orig_len = len;

	D("smd_stream_write() %d -> ch%d\n", len, ch->n);
	if (len < 0) return -EINVAL;

	while ((xfer = ch_write_buffer(ch, &ptr)) != 0) {
		if (!ch_is_open(ch))
			break;
		if (xfer > len)
			xfer = len;
		memcpy(ptr, buf, xfer);
		ch_write_done(ch, xfer);
		len -= xfer;
		buf += xfer;
		if (len == 0)
			break;
	}

	notify_other_smd();

	return orig_len - len;
}

static int smd_packet_write(smd_channel_t *ch, const void *_data, int len)
{
	unsigned hdr[5];

	D("smd_packet_write() %d -> ch%d\n", len, ch->n);
	if (len < 0) return -EINVAL;

	if (smd_stream_write_avail(ch) < (len + SMD_HEADER_SIZE))
		return -ENOMEM;

	hdr[0] = len;
	hdr[1] = hdr[2] = hdr[3] = hdr[4] = 0;

	smd_stream_write(ch, hdr, sizeof(hdr));
	smd_stream_write(ch, _data, len);

	return len;
}

static int smd_stream_read(smd_channel_t *ch, void *data, int len)
{
	int r;

	if (len < 0) return -EINVAL;

	r = ch_read(ch, data, len);
	if (r > 0)
		notify_other_smd();

	return r;
}

static int smd_packet_read(smd_channel_t *ch, void *data, int len)
{
	unsigned long flags;
	int r;

	if (len < 0) return -EINVAL;

	if (len > ch->current_packet)
		len = ch->current_packet;

	r = ch_read(ch, data, len);
	if (r > 0)
		notify_other_smd();

	spin_lock_irqsave(&smd_lock, flags);
	ch->current_packet -= r;
	update_packet_state(ch);
	spin_unlock_irqrestore(&smd_lock, flags);

	return r;
}

static void do_nothing_notify(void *priv, unsigned flags)
{
}

int smd_open(int n, smd_channel_t **_ch, void *priv, void (*notify)(void *, unsigned))
{
	struct smd_channel *ch;
	struct smd_shared *shared;
	unsigned long flags;
	int res = 0;

	if (smd_initialized == 0) {
		printk(KERN_INFO "smd_open() before smd_init()\n");
		return -ENODEV;
	}

	if ((n < 0) || (n >= SMD_CHANNELS))
		return -ENODEV;

	D("smd_open(%d, %p, %p)\n", n, priv, notify);

	if (notify == 0)
		notify = do_nothing_notify;

	mutex_lock(&smd_creation_mutex);

	spin_lock_irqsave(&smd_lock, flags);
	list_for_each_entry(ch, &smd_ch_list, ch_list) {
		if (ch->n == n) {
			res = -ENODEV;
			break;
		}
	}
	spin_unlock_irqrestore(&smd_lock, flags);
	if (res) goto out;

	shared = smem_alloc(ID_SMD_CHANNELS + n, sizeof(struct smd_shared));
	if (shared == 0) {
		printk(KERN_ERR "smd_open() channel %d does not exist\n", n);
		res = -ENODEV;
		goto out;
	}

	ch = kzalloc(sizeof(struct smd_channel), GFP_KERNEL);
	if (ch == 0) {
		res = -ENOMEM;
		goto out;
	}

	ch->send = &shared->ch0;
	ch->recv = &shared->ch1;
	ch->priv = priv;
	ch->notify = notify;
	ch->n = n;
	ch->current_packet = 0;
	ch->last_state = SMD_SS_CLOSED;

	if (smd_is_packet(n)) {
		ch->read = smd_packet_read;
		ch->write = smd_packet_write;
		ch->read_avail = smd_packet_read_avail;
		ch->write_avail = smd_packet_write_avail;
		ch->update_state = update_packet_state;
	} else {
		ch->read = smd_stream_read;
		ch->write = smd_stream_write;
		ch->read_avail = smd_stream_read_avail;
		ch->write_avail = smd_stream_write_avail;
		ch->update_state = update_stream_state;
	}

	*_ch = ch;

	spin_lock_irqsave(&smd_lock, flags);
	list_add(&ch->ch_list, &smd_ch_list);
	D("smd_open: opening ch %d\n", ch->n);
	hc_set_state(ch->send, SMD_SS_OPENED);
	spin_unlock_irqrestore(&smd_lock, flags);
	smd_kick(ch);

	D("smd_open(%d, %p, %p) ch=%p\n", n, priv, notify, ch);

out:
	mutex_unlock(&smd_creation_mutex);
	return res;
}

int smd_close(smd_channel_t *ch)
{
	unsigned long flags;

	printk(KERN_INFO "smd_close(%p)\n", ch);

	if (ch == 0) return -1;

	mutex_lock(&smd_creation_mutex);
	spin_lock_irqsave(&smd_lock, flags);

	/* XXX removing a packet channel may cause desync here */
	list_del(&ch->ch_list);
	D("smd_close: closing ch %d\n", ch->n);
	hc_set_state(ch->send, SMD_SS_CLOSED);

	spin_unlock_irqrestore(&smd_lock, flags);
	mutex_unlock(&smd_creation_mutex);

	return 0;
}

int smd_read(smd_channel_t *ch, void *data, int len)
{
	return ch->read(ch, data, len);
}

int smd_write(smd_channel_t *ch, const void *data, int len)
{
	return ch->write(ch, data, len);
}

int smd_read_avail(smd_channel_t *ch)
{
	return ch->read_avail(ch);
}

int smd_write_avail(smd_channel_t *ch)
{
	return ch->write_avail(ch);
}

int smd_wait_until_readable(smd_channel_t *ch, int bytes)
{
	return -1;
}

int smd_wait_until_writable(smd_channel_t *ch, int bytes)
{
	return -1;
}

int smd_cur_packet_size(smd_channel_t *ch)
{
	return ch->current_packet;
}


/* --------------------------------------------------------------------------------------- */

void *smem_alloc(unsigned id, unsigned size)
{
	return smem_find(id, size);
}

void *smem_find(unsigned id, unsigned size_in)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	struct smem_heap_entry *toc = shared->heap_toc;

	unsigned size = ALIGN(size_in, 8);

	if (id > 128)
		return 0;

	if (toc[id].allocated) {
		if (toc[id].size != size) {
			printk(KERN_ERR "smem_find(%d, %d): wrong size %d, expected %d\n", id, size_in, size, toc[id].size);
			return 0;
		}
		return (void *) (MSM_SHARED_RAM_BASE + toc[id].offset);
	}
	return 0;
}

static irqreturn_t smsm_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct smsm_shared *smsm;

	spin_lock_irqsave(&smem_lock, flags);
	smsm = smem_alloc(ID_SHARED_STATE,
			  2 * sizeof(struct smsm_shared));

	if (smsm == 0) {
		printk(KERN_INFO "<SM NO STATE>\n");
	} else {
		unsigned apps = smsm[0].state;
		unsigned modm = smsm[1].state;

		printk(KERN_INFO "<SM %08x %08x>\n", apps, modm);
		if (modm & SMSM_RESET) {
			smd_diag();
		} else {
			apps |= SMSM_INIT;
			if (modm & SMSM_SMDINIT)
				apps |= SMSM_SMDINIT;
			if (modm & SMSM_RPCINIT)
				apps |= SMSM_RPCINIT;
		}

		if (smsm[0].state != apps) {
			printk(KERN_INFO "<SM %08x NOTIFY>\n", apps);
			smsm[0].state = apps;
			do_smd_probe();
			notify_other_smsm();
		}
	}
	spin_unlock_irqrestore(&smem_lock, flags);
	return IRQ_HANDLED;
}

int smsm_change_state(uint32_t clear_mask, uint32_t set_mask)
{
	unsigned long flags;
	struct smsm_shared *smsm;

	spin_lock_irqsave(&smem_lock, flags);

	smsm = smem_alloc(ID_SHARED_STATE,
			  2 * sizeof(struct smsm_shared));

	if (smsm) {
		smsm[0].state = (smsm[0].state & ~clear_mask) | set_mask;
		printk(KERN_ERR "smsm_change_state %x\n", smsm[0].state);
		notify_other_smsm();
	}

	spin_unlock_irqrestore(&smem_lock, flags);

	if (smsm == NULL) {
		printk(KERN_ERR "smsm_change_state <SM NO STATE>\n");
		return -EIO;
	}
	return 0;
}

uint32_t smsm_get_state(void)
{
	unsigned long flags;
	struct smsm_shared *smsm;
	uint32_t rv;

	spin_lock_irqsave(&smem_lock, flags);

	smsm = smem_alloc(ID_SHARED_STATE,
			  2 * sizeof(struct smsm_shared));

	if (smsm)
		rv = smsm[1].state;
	else
		rv = 0;

	spin_unlock_irqrestore(&smem_lock, flags);

	if (smsm == NULL)
		printk(KERN_ERR "smsm_get_state <SM NO STATE>\n");
	return rv;
}

int smsm_set_sleep_duration(uint32_t delay)
{
	uint32_t *ptr;

	ptr = smem_alloc(SMEM_SMSM_SLEEP_DELAY, sizeof(*ptr));
	if (ptr == NULL) {
		printk(KERN_ERR "smsm_set_sleep_duration <SM NO SLEEP_DELAY>\n");
		return -EIO;
	}
	printk(KERN_INFO "smsm_set_sleep_duration %d -> %d\n", *ptr, delay);
	*ptr = delay;
	return 0;
}

int smsm_set_interrupt_info(struct smsm_interrupt_info *info)
{
	struct smsm_interrupt_info *ptr;

	ptr = smem_alloc(SMEM_SMSM_INT_INFO, sizeof(*ptr));
	if (ptr == NULL) {
		printk(KERN_ERR "smsm_set_sleep_duration <SM NO INT_INFO>\n");
		return -EIO;
	}
	printk(KERN_INFO "smsm_set_interrupt_info %x %x -> %x %x\n",
	       ptr->aArm_en_mask, ptr->aArm_interrupts_pending,
	       info->aArm_en_mask, info->aArm_interrupts_pending);
	*ptr = *info;
	return 0;
}

#define MAX_NUM_SLEEP_CLIENTS       64
#define MAX_SLEEP_NAME_LEN          8

#define NUM_GPIO_INT_REGISTERS 6
struct tramp_gpio_save
{
  unsigned int enable;
  unsigned int detect;
  unsigned int polarity;
};
struct tramp_gpio_smem
{
  struct tramp_gpio_save settings[NUM_GPIO_INT_REGISTERS];
  unsigned int         fired[NUM_GPIO_INT_REGISTERS];
  unsigned int         group;
};


void smsm_print_sleep_info(void)
{
	unsigned long flags;
	uint32_t *ptr;
	struct tramp_gpio_smem *gpio;
	struct smsm_interrupt_info *int_info;


	spin_lock_irqsave(&smem_lock, flags);

	ptr = smem_alloc(SMEM_SMSM_SLEEP_DELAY, sizeof(*ptr));
	if (ptr)
		printk(KERN_ERR "SMEM_SMSM_SLEEP_DELAY: %x\n", *ptr);
	else
		printk(KERN_ERR "SMEM_SMSM_SLEEP_DELAY: missing\n");

	ptr = smem_alloc(SMEM_SMSM_LIMIT_SLEEP, sizeof(*ptr));
	if (ptr)
		printk(KERN_ERR "SMEM_SMSM_LIMIT_SLEEP: %x\n", *ptr);
	else
		printk(KERN_ERR "SMEM_SMSM_LIMIT_SLEEP: missing\n");

	ptr = smem_alloc(SMEM_SLEEP_POWER_COLLAPSE_DISABLED, sizeof(*ptr));
	if (ptr)
		printk(KERN_ERR "SMEM_SLEEP_POWER_COLLAPSE_DISABLED: %x\n", *ptr);
	else
		printk(KERN_ERR "SMEM_SLEEP_POWER_COLLAPSE_DISABLED: missing\n");

	int_info = smem_alloc(SMEM_SMSM_INT_INFO, sizeof(*ptr));
	if (int_info)
		printk(KERN_INFO "SMEM_SMSM_INT_INFO %x %x\n",
		       int_info->aArm_en_mask, int_info->aArm_interrupts_pending);
	else
		printk(KERN_ERR "SMEM_SMSM_INT_INFO: missing\n");

	gpio = smem_alloc( SMEM_GPIO_INT, sizeof(*gpio)); 
	if (gpio) {
		int i;
		for(i = 0; i < ARRAY_SIZE(gpio->settings); i++) {
			printk(KERN_ERR "SMEM_GPIO_INT: %d: e %x d %x p %x f %x\n",
			       i, gpio->settings[i].enable, gpio->settings[i].detect,
			       gpio->settings[i].polarity, gpio->fired[i]);
		}
		printk(KERN_ERR "SMEM_GPIO_INT: group %x\n", gpio->group);
	}
	else
		printk(KERN_ERR "SMEM_GPIO_INT: missing\n");
	
#if 0
	ptr = smem_alloc( SMEM_SLEEP_STATIC, 
                                           2 * MAX_NUM_SLEEP_CLIENTS * 
                                           ( MAX_SLEEP_NAME_LEN + 1 ) ); 
	if (ptr)
		printk(KERN_ERR "SMEM_SLEEP_STATIC: %x %x %x %x\n", ptr[0], ptr[1], ptr[2], ptr[3]);
	else
		printk(KERN_ERR "SMEM_SLEEP_STATIC: missing\n");
#endif

	spin_unlock_irqrestore(&smem_lock, flags);
}

int smd_core_init(void)
{
	int r;
	printk(KERN_INFO "smd_core_init()\n");

	r = request_irq(INT_A9_M2A_0, smd_irq_handler,
			IRQF_TRIGGER_RISING, "smd_dev", 0);
	if (r < 0)
		return r;
	r = enable_irq_wake(INT_A9_M2A_0);
	if (r < 0)
		printk(KERN_ERR "smd_core_init: enable_irq_wake failed for INT_A9_M2A_0\n");

	r = request_irq(INT_A9_M2A_5, smsm_irq_handler,
			IRQF_TRIGGER_RISING, "smsm_dev", 0);
	if (r < 0) {
		free_irq(INT_A9_M2A_0, 0);
		return r;
	}
	r = enable_irq_wake(INT_A9_M2A_5);
	if (r < 0)
		printk(KERN_ERR "smd_core_init: enable_irq_wake failed for INT_A9_M2A_5\n");

	/* we may have missed a signal while booting -- fake
	 * an interrupt to make sure we process any existing
	 * state
	 */
	smsm_irq_handler(0, 0);

	printk(KERN_INFO "smd_core_init() done\n");

	return 0;
}

#if defined(CONFIG_DEBUG_FS)

static int dump_ch(char *buf, int max, int n,
		  struct smd_half_channel *s,
		  struct smd_half_channel *r)
{
	return scnprintf(
		buf, max,
		"ch%02d:"
		" %8s(%04d/%04d) %c%c%c%c%c%c%c <->"
		" %8s(%04d/%04d) %c%c%c%c%c%c%c\n", n,
		chstate(s->state), s->tail, s->head,
		s->fDSR ? 'D' : 'd',
		s->fCTS ? 'C' : 'c',
		s->fCD ? 'C' : 'c',
		s->fRI ? 'I' : 'i',
		s->fHEAD ? 'W' : 'w',
		s->fTAIL ? 'R' : 'r',
		s->fSTATE ? 'S' : 's',
		chstate(r->state), r->tail, r->head,
		r->fDSR ? 'D' : 'd',
		r->fCTS ? 'R' : 'r',
		r->fCD ? 'C' : 'c',
		r->fRI ? 'I' : 'i',
		r->fHEAD ? 'W' : 'w',
		r->fTAIL ? 'R' : 'r',
		r->fSTATE ? 'S' : 's'
		);
}

static int debug_read_stat(char *buf, int max)
{
	struct smsm_shared *smsm;
	char *msg;
	int i = 0;

	smsm = smem_find(ID_SHARED_STATE,
			 2 * sizeof(struct smsm_shared));

	msg = smem_find(ID_DIAG_ERR_MSG, SZ_DIAG_ERR_MSG);

	if (smsm) {
		if (smsm[1].state & SMSM_RESET)
			i += scnprintf(buf + i, max - i, "smsm: ARM9 HAS CRASHED\n");
		i += scnprintf(buf + i, max - i, "smsm: a9: %08x a11: %08x\n",
			       smsm[0].state, smsm[1].state);
	} else {
		i += scnprintf(buf + i, max - i, "smsm: cannot find\n");
	}
	if (msg) {
		msg[SZ_DIAG_ERR_MSG - 1] = 0;
		i += scnprintf(buf + i, max - i, "diag: '%s'\n", msg);
	}
	return i;
}

static int debug_read_mem(char *buf, int max)
{
	unsigned n;
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	struct smem_heap_entry *toc = shared->heap_toc;
	int i = 0;

	i += scnprintf(buf + i, max - i,
		       "heap: init=%d free=%d remain=%d\n",
		       shared->heap_info.initialized,
		       shared->heap_info.free_offset,
		       shared->heap_info.heap_remaining);

	for (n = 0; n < 128; n++) {
		if (toc[n].allocated == 0)
			continue;
		i += scnprintf(buf + i, max - i,
			       "%04d: offsed %08x size %08x\n",
			       n, toc[n].offset, toc[n].size);
	}
	return i;
}

static int debug_read_ch(char *buf, int max)
{
	struct smd_shared *shared;
	int n, i = 0;

	for (n = 0; n < SMD_CHANNELS; n++) {
		shared = smem_find(ID_SMD_CHANNELS + n, sizeof(struct smd_shared));
		if (shared == 0)
			continue;
		i += dump_ch(buf + i, max - i, n, &shared->ch0, &shared->ch1);
	}

	return i;
}

static int debug_read_version(char *buf, int max)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	unsigned version = shared->version[VERSION_MODEM];
	return sprintf(buf, "%d.%d\n", version >> 16, version & 0xffff);
}


#define DEBUG_BUFMAX 4096
static char debug_buffer[DEBUG_BUFMAX];

static ssize_t debug_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, DEBUG_BUFMAX);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
};

static void debug_create(const char *name, mode_t mode,
			 struct dentry *dent,
			 int (*fill)(char *buf, int max))
{
	debugfs_create_file(name, mode, dent, fill, &debug_ops);
}

static void smd_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("smd", 0);
	if (IS_ERR(dent))
		return;

	debug_create("ch", 0444, dent, debug_read_ch);
	debug_create("stat", 0444, dent, debug_read_stat);
	debug_create("mem", 0444, dent, debug_read_mem);
	debug_create("version", 0444, dent, debug_read_version);
}
#else
static void smd_debugfs_init(void) {}
#endif

static int __init msm_smd_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "smd_init()\n");

	INIT_WORK(&probe_work, smd_channel_probe_worker);

	if (smd_core_init()) {
		printk(KERN_ERR "smd_core_init() failed\n");
		return -1;
	}

	memset(&p_devs, 0, sizeof(p_devs));
	do_smd_probe();

	smd_debugfs_init();
	smd_initialized = 1;

	return 0;
}

static struct platform_driver msm_smd_driver = {
	.probe = msm_smd_probe,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_smd_init(void)
{
	return platform_driver_register(&msm_smd_driver);
}

module_init(msm_smd_init);

MODULE_DESCRIPTION("MSM Shared Memory Core");
MODULE_AUTHOR("Brian Swetland <swetland@google.com>");
MODULE_LICENSE("GPL");
