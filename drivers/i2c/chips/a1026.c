/* drivers/i2c/chips/a1026.c - a1026 voice processor driver
 *
 * Copyright (C) 2009 HTC Corporation.
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/a1026.h>

#define DEBUG	0

static struct i2c_client *this_client;
static struct a1026_platform_data *pdata;

static int execute_cmdmsg(unsigned int);
static struct mutex a1026_lock;
static int a1026_opened;
static int A1026_Suspended;

struct vp_ctxt {
	unsigned char *data;
	unsigned int img_size;
};

struct vp_ctxt the_vp;

static int A1026I2C_RxData(char *rxData, int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msgs, 1) < 0) {
		pr_err("A1026 RxData: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int A1026I2C_TxData(char *txData, int length)
{

	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
		pr_err("A1026 TxData: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int Read_A1026_Data_Bytes(unsigned char *buf, unsigned char len)
{
	int ret = 0;

	ret = A1026I2C_RxData(buf, len);
#if DEBUG
	int i = 0;
	for (i = 0; i < len; i++)
	{
		pr_info("A1026 i2c recv[%d] = 0x%2x\n", i , *(buf + i));
	}
#endif
	return ret;
}

static int a1026_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct vp_ctxt *vp = &the_vp;

	mutex_lock(&a1026_lock);

	if (a1026_opened) {
		pr_err("a1026: busy\n");
		rc = -EBUSY;
		goto done;
	}

	file->private_data = vp;
	vp->img_size = 0;
	a1026_opened = 1;
done:
	mutex_unlock(&a1026_lock);
	return rc;
}

static int a1026_release(struct inode *inode, struct file *file)
{
	mutex_lock(&a1026_lock);
	a1026_opened = 0;
	mutex_unlock(&a1026_lock);

	return 0;
}

static ssize_t a1026_bootup_init(struct file *file, struct a1026img *img)
{
	struct vp_ctxt *vp = file->private_data;
	int rc, pass = 0;
	int size, size2I2C = 0;
	int retry = RETRY_CNT;
	unsigned char *index;
	char buf[2];
	char sync_msg[4];
	unsigned char buf32[32];
	int polling_retry_cnt = 0;

	if (img->img_size < 0) {
		rc = -EINVAL;
		pr_err("Invalid A1026 image size (%d)\n", rc);
		return rc;
	}

	while(retry--) {
		/* Init for while loop */
		rc = 0;

		/* Reset A1026 chip */
		gpio_set_value(pdata->gpio_a1026_reset, 0);
		mdelay(1);
		gpio_set_value(pdata->gpio_a1026_reset, 1);
		mdelay(20); /* Delay before send I2C command */

		/* Boot Cmd to A1026 */
		buf[0] = A1026_msg_BOOT>>8;
		buf[1] = A1026_msg_BOOT&0xff;

		rc = A1026I2C_TxData(buf, 2);
		if (rc < 0) {
			pr_err("Set boot mode error (retry = %d)\n", retry);
			continue;
		}

		mdelay(20); /* use polling */
		rc = Read_A1026_Data_Bytes(buf, 1);
		if (rc < 0) {
			pr_err("Read boot mode Ack error (retry = %d)\n",
				retry);
			continue;
		}

		if (buf[0] != A1026_msg_BOOT_ACK) {
			pr_err("A1026 failed to set boot mode (retry = %d)\n",
				retry);
			continue;
		} else {
			vp->data = kmalloc(img->img_size, GFP_KERNEL);
			if (copy_from_user(vp->data, img->buf, img->img_size)) {
				rc = -EFAULT;
				pr_err("copy error (rc = %d, retry = %d)\n",
					rc, retry);
				kfree(vp->data);
				continue;
			}

			vp->img_size = img->img_size;

			pr_info("A1026 start to load image...\n");

			size = vp->img_size;
			index = vp->data;

			while (size > 0) {
				memset(buf32, 0,sizeof(buf32));

				if ((size - 32) >= 0) {
					size2I2C = 32;
					size -= 32;
				} else {
					size2I2C = size;
					size -= size;
				}
				memcpy(buf32, index, size2I2C);
				if (size)
					index += size2I2C;

				rc = A1026I2C_TxData(buf32, size2I2C);
				if (rc < 0)
					break;
			}
			if (rc < 0) {
				pr_err("Tx A1026 img error (retry = %d)\n",
					retry);
				continue;
			}
		}

		mdelay(20); /* Delay time before issue a Sync Cmd */

		/* Issue Sync Cmd(0x80000000) to A1026 */
		memset(sync_msg, 0, sizeof(sync_msg));
		sync_msg[0] = 0x80;
		sync_msg[1] = 0x00;
		sync_msg[2] = 0x00;
		sync_msg[3] = 0x00;
		rc = A1026I2C_TxData(sync_msg, 4);
		if (rc < 0) {
			pr_err("A1026 Tx Sync Cmd error (retry = %d)\n",
				retry);
			continue;
		}
		polling_retry_cnt = POLLING_RETRY_CNT;
retry_polling:
		mdelay(20); /* use polling */
		memset(sync_msg, 0, sizeof(sync_msg));
		rc = Read_A1026_Data_Bytes(sync_msg, 4);
		if (rc < 0) {
			pr_err("A1026 Read Sync cmd Ack error (retry = %d)\n",
				retry);
			continue;
		}

		if (sync_msg[0] == 0x80 && sync_msg[1] == 0x00
		&& sync_msg[2] == 0x00 && sync_msg[3] == 0x00) {
			pass = 1;
			break;
		} else if (polling_retry_cnt) {
			polling_retry_cnt--;
			pr_info("A1026 polling_retry_cnt = %d\n", polling_retry_cnt);
			goto retry_polling;
		} else {
			pr_err("A1026 Sync Cmd ACK NG (retry = %d)\n",
				retry);
			rc = -1;
			continue;
		}
	}

	/* Put A1026 into sleep mode */
	rc = execute_cmdmsg(A100_msg_Sleep);
	if (rc < 0) {
		pr_err("A1026 Suspend Error\n");
		goto set_suspend_err;
	}
	A1026_Suspended = 1;

set_suspend_err:
	if (pass == 1 && !rc) {
		pr_info("A1026 Boot Up Init Completed!\n");
		kfree(vp->data);
	} else
		pr_err("A1026 Fatal Error!!! Cannot load firmware image\n");

	return rc;
}

unsigned int phonecall_receiver[] = {
	0x80260001, // Select audio routing 1
	0x80170002,
	0x80180000, // Select CT Mic configuration
	0x800C0300,
	0x800D0001, // Set ADC0 gain to +6dB
	0x800C0400,
	0x800D0001, // Set ADC1 gain to +6dB
	0x801B0000, // Set Digital input gain to 0dB
	0x80150000, // Set Digital output gain to 0dB
	0x80230000, // Set Downlink digital gain to 0dB
	0x801C0000, // Set Voice Processing off (bypass mode)
};

unsigned int phonecall_headset[] = {
	0x80260015, // Select audio routing 21
	0x80170002,
	0x80180000, // Select CT Mic configuration
	0x800C0400,
	0x800D0001, // Set ADC1 gain to +6dB
	0x801B0000, // Set Digital input gain to 0dB
	0x80150000, // Set Digital output gain to 0dB
	0x80230000, // Set Downlink digital gain to 0dB
	0x801C0000, // Set Voice Processing off (bypass mode)
};

unsigned int phonecall_speaker[] = {
	0x80260007, // Select audio routing 7
	0x80170002,
	0x80180002, // Select FT Mic configuration
	0x800C0300,
	0x800D0002, // Set ADC0 gain to +12dB
	0x801B0000, // Set Digital input gain to 0dB
	0x80150000, // Set Digital output gain to 0dB
	0x80230000, // Set Downlink digital gain to 0dB
	0x801C0000, // Set Voice Processing off (bypass mode)
};

unsigned int phonecall_bt[] = {
	0x80260006, // Select audio routing 6
	0x801B0000, // Set Digital input gain to 0dB
	0x80150000, // Set Digital output gain to 0dB
	0x80230000, // Set Downlink digital gain to 0dB
	0x801C0000, // Set Voice Processing off (bypass mode)
};

unsigned int INT_MIC_recording[] = {
	0x80260007, // Select audio routing 7
	0x800C0300,
	0x800D0002, // Set ADC0 gain to +12dB
	0x801B0000, // Set Digital input gain to 0dB
	0x80150000, // Set Digital output gain to 0dB
	0x80230000, // Set Downlink digital gain to 0dB
	0x801C0000, // Set Voice Processing off (bypass mode)
};

unsigned int EXT_MIC_recording[] = {
	0x80260015, // Select audio routing 21
	0x80170002,
	0x80180000, // Select CT Mic configuration
	0x800C0400,
	0x800D0001, // Set ADC1 gain to +6dB
	0x801B0000, // Set Digital input gain to 0dB
	0x80150000, // Set Digital output gain to 0dB
	0x80230000, // Set Downlink digital gain to 0dB
	0x801C0000, // Set Voice Processing off (bypass mode)
};

unsigned int CAM_coder_recording[] = {
	0x80260007, // Select audio routing 7
	0x80170002,
	0x80180002, // Select FT Mic configuration
	0x800C0300,
	0x800D0002, // Set ADC0 gain to +12dB
	0x801B0000, // Set Digital input gain to 0dB
	0x80150000, // Set Digital output gain to 0dB
	0x80230000, // Set Downlink digital gain to 0dB
	0x801C0000, // Set Voice Processing off (bypass mode)
};

unsigned int suspend_mode[] = {
	A100_msg_Sleep,
};

static ssize_t chk_wakeup_a1026(void)
{
	int rc = 0;

	if (A1026_Suspended == 1) {
		gpio_set_value(pdata->gpio_a1026_wakeup, 0);
		mdelay(10);
		gpio_set_value(pdata->gpio_a1026_wakeup, 1);
		mdelay(1); /* Check it with Audience */
		rc = execute_cmdmsg(0x80000000); /* issue a Sync CMD */
		if (rc < 0) {
			pr_err("Wakeup A1026 Failed!\n");
			return rc;
		}
		A1026_Suspended = 0;
	}
	return rc;
}

int a1026_set_config(char newid)
{
	int i = 0, rc = 0;
	struct cmd_list new_list;

	if ((A1026_Suspended) && (newid == A1026_PATH_SUSPEND))
		return rc;

	rc = chk_wakeup_a1026();
	if(rc < 0)
		return rc;

	switch (newid) {
	case A1026_PATH_INCALL_RECEIVER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = phonecall_receiver;
		new_list.cnt = sizeof(phonecall_receiver)/sizeof(unsigned int);
		break;
	case A1026_PATH_INCALL_HEADSET:
		gpio_set_value(pdata->gpio_a1026_micsel, 1);
		new_list.p = phonecall_headset;
		new_list.cnt = sizeof(phonecall_headset)/sizeof(unsigned int);
		break;
	case A1026_PATH_INCALL_SPEAKER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = phonecall_speaker;
		new_list.cnt = sizeof(phonecall_speaker)/sizeof(unsigned int);
		break;
	case A1026_PATH_INCALL_BT:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = phonecall_bt;
		new_list.cnt = sizeof(phonecall_bt)/sizeof(unsigned int);
		break;
	case A1026_PATH_INCALL_VR_RECEIVER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = phonecall_receiver; /*TBD*/
		new_list.cnt = sizeof(phonecall_receiver)/sizeof(unsigned int);
		break;
	case A1026_PATH_INCALL_VR_HEADSET:
		gpio_set_value(pdata->gpio_a1026_micsel, 1);
		new_list.p = phonecall_headset; /*TBD*/
		new_list.cnt = sizeof(phonecall_headset)/sizeof(unsigned int);
		break;
	case A1026_PATH_INCALL_VR_SPEAKER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = phonecall_speaker; /*TBD*/
		new_list.cnt = sizeof(phonecall_speaker)/sizeof(unsigned int);
		break;
	case A1026_PATH_INCALL_VR_BT:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = phonecall_bt; /*TBD*/
		new_list.cnt = sizeof(phonecall_bt)/sizeof(unsigned int);
		break;
	case A1026_PATH_RECORD_RECEIVER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = INT_MIC_recording;
		new_list.cnt = sizeof(INT_MIC_recording)/sizeof(unsigned int);
		break;
	case A1026_PATH_RECORD_HEADSET:
		gpio_set_value(pdata->gpio_a1026_micsel, 1);
		new_list.p = EXT_MIC_recording;
		new_list.cnt = sizeof(EXT_MIC_recording)/sizeof(unsigned int);
		break;
	case A1026_PATH_RECORD_SPEAKER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = CAM_coder_recording;
		new_list.cnt = sizeof(CAM_coder_recording)/sizeof(unsigned int);
		break;
	case A1026_PATH_RECORD_BT:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = phonecall_bt; /*TBD*/
		new_list.cnt = sizeof(phonecall_bt)/sizeof(unsigned int);
		break;
	case A1026_PATH_SUSPEND:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = suspend_mode;
		new_list.cnt = sizeof(suspend_mode)/sizeof(unsigned int);
		break;
	default:
		pr_err("A1026 set config: Invalid input\n");
		rc = -1;
		goto input_err;
		break;
	}

	pr_info("A1026 change mode command count = %d\n", new_list.cnt);
	for (i = 0; i < new_list.cnt; i++) {
		pr_info("A1026 cmds: i = %d, *p = 0x%x\n", i, *(new_list.p));
		rc = execute_cmdmsg(*(new_list.p));
		if (rc < 0)
			break;
		if (*(new_list.p) == A100_msg_Sleep)
			A1026_Suspended = 1;
		new_list.p++;
	}

	if (rc < 0)
		pr_err("Exe cmd-%d error...(0x%x)\n", i, *(new_list.p));
input_err:
	return rc;
}

int execute_cmdmsg(unsigned int msg)
{
	int rc = 0;
	int exe_retry, exe_pass = 0;
	unsigned char msgbuf[4];
	unsigned char chkbuf[4];

	memset(msgbuf, 0, sizeof(msgbuf));
	memset(chkbuf, 0, sizeof(chkbuf));

	msgbuf[0] = (msg >> 24) & 0xFF;
	msgbuf[1] = (msg >> 16) & 0xFF;
	msgbuf[2] = (msg >> 8) & 0xFF;
	msgbuf[3] = msg & 0xFF;

	memcpy(chkbuf, msgbuf, 4);

	rc = A1026I2C_TxData(msgbuf, 4);
	if (rc < 0) {
		pr_err("Tx A1026 Exe Cmd error\n");
		return rc;
	}
	exe_retry = POLLING_RETRY_CNT;

	while (exe_retry--) {
		rc = 0;

		mdelay(20); /* use polling */
		memset(msgbuf, 0, sizeof(msgbuf));
		rc = Read_A1026_Data_Bytes(msgbuf, 4);
		if (rc < 0) {
			pr_err("A1026 Read Exe cmd Ack error (%d)\n", exe_retry);
			continue;
		}

		if (msgbuf[0] == chkbuf[0] && msgbuf[1] == chkbuf[1]
		&& msgbuf[2] == chkbuf[2] && msgbuf[3] == chkbuf[3]) {
			exe_pass = 1;
			break;
		} else {
			pr_info("A1026 exe_retry cnt = %d\n", exe_retry);
		}
	}

	if (!exe_pass) {
		pr_err("A1026 Exe Cmd ACK NG\n");
		rc = -1;
	}
	return rc;
}

static int Set_A1026_MIC_ONOFF_BY_CASE(char miccase)
{
	int rc = 0;
	unsigned int cmd_msg = 0;

	switch (miccase) {
	case 1: /* Mic-1 ON / Mic-2 OFF */
		cmd_msg = 0x80260007;
		break;
	case 2: /* Mic-1 OFF / Mic-2 ON */
		cmd_msg = 0x80260015;
		break;
	case 3: /* both ON */
		cmd_msg = 0x80260001;
		break;
	case 4: /* both OFF */
		cmd_msg = 0x80260006;
		break;
	default:
		pr_info("A1026 Invalid input case number\n");
		rc = -EINVAL;
		break;
	}
	rc = execute_cmdmsg(cmd_msg);
	return rc;
}

static int exe_cmd_in_file(unsigned char *incmd)
{
	int rc = 0;
	int i = 0;
	unsigned int cmd_msg = 0;
	unsigned char tmp = 0;

	for (i = 0; i < 4; i++) {
		tmp = *(incmd + i);
		cmd_msg |= (unsigned int)tmp;
		if (i != 3) {
			cmd_msg = cmd_msg << 8;
		}
	}
	rc = execute_cmdmsg(cmd_msg);
	if (rc < 0)
		pr_err("A1026 Exe cmd error...(0x%x)\n", cmd_msg);
	return rc;
}

static int
a1026_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct a1026img img;
	int ret = 0;
	char msg[4];
	char mic_cases = 0;
	char mic_sel = 0;
	char pathid = 0;

	switch (cmd) {
	case A1026_BOOTUP_INIT:
		img.buf = 0;
		img.img_size = 0;
		if (copy_from_user(&img, argp, sizeof(img)))
			return -EFAULT;
		ret = a1026_bootup_init(file, &img);
		break;
	case A1026_SET_CONFIG:
		if (copy_from_user(&pathid, argp, sizeof(pathid)))
			return -EFAULT;
		ret = a1026_set_config(pathid);
		if (ret < 0)
			pr_err("A1026_SET_CONFIG (%d) NG!\n", pathid);
		break;
	case A1026_SET_MIC_ONOFF:
		ret = chk_wakeup_a1026();
		if(ret < 0)
			return ret;
		if (copy_from_user(&mic_cases, argp, sizeof(mic_cases)))
			return -EFAULT;
		ret = Set_A1026_MIC_ONOFF_BY_CASE(mic_cases);
		if (ret < 0)
			pr_err("A1026_SET_MIC_ONOFF (%d) NG!\n", mic_cases);
		break;
	case A1026_SET_MICSEL_ONOFF:
		ret = chk_wakeup_a1026();
		if(ret < 0)
			return ret;
		if (copy_from_user(&mic_sel, argp, sizeof(mic_sel)))
			return -EFAULT;
		if ((mic_sel < 0) || (mic_sel > 1))
			return -EINVAL;
		if (mic_sel == 0)
			gpio_set_value(pdata->gpio_a1026_micsel, 0);
		else
			gpio_set_value(pdata->gpio_a1026_micsel, 1);
		ret = 0;
		break;
	case A1026_READ_DATA:
		ret = chk_wakeup_a1026();
		if(ret < 0)
			return ret;
		ret = Read_A1026_Data_Bytes(msg, 4);
		if (copy_to_user(argp, &msg, 4))
			return -EFAULT;
		break;
	case A1026_SYNC_CMD:
		ret = chk_wakeup_a1026();
		if(ret < 0)
			return ret;
		msg[0] = 0x80;
		msg[1] = 0x00;
		msg[2] = 0x00;
		msg[3] = 0x00;
		ret = A1026I2C_TxData(msg, 4);
		break;
	case A1026_SET_CMD_FILE:
		ret = chk_wakeup_a1026();
		if(ret < 0)
			return ret;
		if (copy_from_user(msg, argp, sizeof(msg)))
			return -EFAULT;
		ret = exe_cmd_in_file(msg);
		break;
	default:
		pr_err("ioctl command not supported!\n");
		ret = -1;
		break;
	}

	return ret;
}

static struct file_operations a1026_fops = {
	.owner = THIS_MODULE,
	.open = a1026_open,
	.release = a1026_release,
	.ioctl = a1026_ioctl,
};

static struct miscdevice a1026_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "audience_a1026",
	.fops = &a1026_fops,
};

static int a1026_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	pdata = client->dev.platform_data;

	if (pdata == NULL) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			ret = -ENOMEM;
			pr_err("A1026: platform data is NULL\n");
			goto err_alloc_data_failed;
		}
	}

	this_client = client;

	ret = gpio_request(pdata->gpio_a1026_micsel, "a1026");
	if (ret < 0) {
		pr_err("A1026: gpio request mic_sel pin failed\n");
		goto err_free_gpio_micsel;
	}

	ret = gpio_direction_output(pdata->gpio_a1026_micsel, 1);
	if (ret < 0) {
		pr_err("A1026: request mic_sel gpio direction failed\n");
		goto err_free_gpio_micsel;
	}

	ret = gpio_request(pdata->gpio_a1026_wakeup, "a1026");
	if (ret < 0) {
		pr_err("A1026: gpio request wakeup pin failed\n");
		goto err_free_gpio;
	}

	ret = gpio_direction_output(pdata->gpio_a1026_wakeup, 1);
	if (ret < 0) {
		pr_err("A1026: request wakeup gpio direction failed\n");
		goto err_free_gpio;
	}

	ret = gpio_request(pdata->gpio_a1026_reset, "a1026");
	if (ret < 0) {
		pr_err("A1026: gpio request reset pin failed\n");
		goto err_free_gpio;
	}

	ret = gpio_direction_output(pdata->gpio_a1026_reset, 1);
	if (ret < 0) {
		pr_err("A1026: request reset gpio direction failed\n");
		goto err_free_gpio_all;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("A1026: i2c check functionality error\n");
		ret = -ENODEV;
		goto err_free_gpio_all;
	}

	gpio_set_value(pdata->gpio_a1026_micsel, 0);
	gpio_set_value(pdata->gpio_a1026_wakeup, 1);
	gpio_set_value(pdata->gpio_a1026_reset, 1);

	ret = misc_register(&a1026_device);
	if (ret) {
		pr_err("a1026_probe: a1026_device register failed\n");
		goto err_free_gpio_all;
	}

	return 0;

err_free_gpio_all:
	gpio_free(pdata->gpio_a1026_reset);
err_free_gpio:
	gpio_free(pdata->gpio_a1026_wakeup);
err_free_gpio_micsel:
	gpio_free(pdata->gpio_a1026_micsel);
err_alloc_data_failed:
	return ret;
}

static int a1026_remove(struct i2c_client *client)
{
	struct a1026_platform_data *p1026data = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(p1026data);

	return 0;
}

static int a1026_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int a1026_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id a1026_id[] = {
	{ "audience_a1026", 0 },
	{ }
};

static struct i2c_driver a1026_driver = {
	.probe = a1026_probe,
	.remove = a1026_remove,
	.suspend = a1026_suspend,
	.resume	= a1026_resume,
	.id_table = a1026_id,
	.driver = {
		.name = "audience_a1026",
	},
};

static int __init a1026_init(void)
{
	pr_info("A1026 Voice Processor driver: init\n");
	mutex_init(&a1026_lock);

	return i2c_add_driver(&a1026_driver);
}

static void __exit a1026_exit(void)
{
	i2c_del_driver(&a1026_driver);
}

module_init(a1026_init);
module_exit(a1026_exit);

MODULE_DESCRIPTION("A1026 voice processor driver");
MODULE_LICENSE("GPL");
