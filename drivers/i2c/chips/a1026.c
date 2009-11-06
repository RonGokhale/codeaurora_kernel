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
static int control_a1026_clk = 0;
static unsigned int a1026_NS_state = A1026_NS_STATE_AUTO;
static int a1026_current_config = A1026_PATH_SUSPEND;
static int a1026_param_ID;

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

static void A1026_sw_reset(unsigned int reset_cmd)
{
	int rc = 0;
	unsigned char msgbuf[4];

	msgbuf[0] = (reset_cmd >> 24) & 0xFF;
	msgbuf[1] = (reset_cmd >> 16) & 0xFF;
	msgbuf[2] = (reset_cmd >> 8) & 0xFF;
	msgbuf[3] = reset_cmd & 0xFF;

	pr_info("Do A1026 Software Reset +++++ (0x%x)\n", reset_cmd);
	rc = A1026I2C_TxData(msgbuf, 4);
	pr_info("Do A1026 Software Reset -----\n");
	if (!rc)
		mdelay(20);
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

		/* Enable A1026 clock */
		if (control_a1026_clk)
			gpio_set_value(pdata->gpio_a1026_clk, 1);

		mdelay(1);
		gpio_set_value(pdata->gpio_a1026_reset, 1);
		mdelay(50); /* Delay before send I2C command */

		/* Boot Cmd to A1026 */
		buf[0] = A1026_msg_BOOT>>8;
		buf[1] = A1026_msg_BOOT&0xff;

		rc = A1026I2C_TxData(buf, 2);
		if (rc < 0) {
			pr_err("Set boot mode error (%d retry cnt left)\n",
				retry);
			continue;
		}

		mdelay(20); /* use polling */
		rc = Read_A1026_Data_Bytes(buf, 1);
		if (rc < 0) {
			pr_err("Read boot mode Ack error (%d retry cnt left)\n",
				retry);
			continue;
		}

		if (buf[0] != A1026_msg_BOOT_ACK) {
			pr_err("A1026 failed to set boot mode (%d retry cnt left)\n",
				retry);
			continue;
		} else {
			vp->data = kmalloc(img->img_size, GFP_KERNEL);
			if (copy_from_user(vp->data, img->buf, img->img_size)) {
				rc = -EFAULT;
				pr_err("copy error (rc = %d, %d retry cnt left)\n",
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
				pr_err("Tx A1026 img error (%d retry cnt left)\n",
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
			pr_err("A1026 Tx Sync Cmd error (%d retry cnt left)\n",
				retry);
			continue;
		}
		polling_retry_cnt = POLLING_RETRY_CNT;
retry_polling:
		mdelay(20); /* use polling */
		memset(sync_msg, 0, sizeof(sync_msg));
		rc = Read_A1026_Data_Bytes(sync_msg, 4);
		if (rc < 0) {
			pr_err("A1026 Read Sync cmd Ack error (%d retry cnt left)\n",
				retry);
			continue;
		}

		if (sync_msg[0] == 0x80 && sync_msg[1] == 0x00
		&& sync_msg[2] == 0x00 && sync_msg[3] == 0x00) {
			pass = 1;
			break;
		} else if (polling_retry_cnt) {
			polling_retry_cnt--;
			pr_info("A1026 polling_retry_cnt left = %d\n",
				polling_retry_cnt);
			goto retry_polling;
		} else {
			pr_err("A1026 Sync Cmd ACK NG (%d retry cnt left)\n",
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
	a1026_current_config = A1026_PATH_SUSPEND;

	mdelay(120);
	/* Disable A1026 clock */
	if (control_a1026_clk)
		gpio_set_value(pdata->gpio_a1026_clk, 0);

set_suspend_err:
	if (pass == 1 && !rc)
		pr_info("A1026 Boot Up Init Completed!\n");
	else
		pr_err("A1026 Fatal Error!!! Cannot load firmware image\n");

	kfree(vp->data);
	return rc;
}

unsigned int phonecall_receiver[] = {
	0x80000000, //0x8000:Sync, 0x0000:None
	0x80260001, //0x8026:SelectRouting, 0x0001:Pri,Sec,FEi,Lp1 -- CSp,FEo,Lp1
	0x80170002, //0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration
	0x80180000, //0x8018:SetAlgorithmParm, 0x0000:Close Talk (CT)
	0x8017001A, //0x8017:SetAlgorithmParmID, 0x001A:Use ComfortNoise
	0x80180000, //0x8018:SetAlgorithmParm, 0x0000:No
	0x801C0001, //0x801C:VoiceProcessingOn, 0x0001:Yes
	0x800C0300, //0x800C:SetDeviceParmID, 0x03:ADC0, 0x00:ADC Gain
	0x800D0001, //0x800D:SetDeviceParm, 0x0001:+ 6dB
	0x800C0400, //0x800C:SetDeviceParmID, 0x04:ADC1, 0x00:ADC Gain
	0x800D0001, //0x800D:SetDeviceParm, 0x0001:+ 6dB
	0x801B000C, //0x801B:SetDigitalInputGain, 0x00:Primay Mic (Tx), 0x0C:(12 dB)
	0x801B010C, //0x801B:SetDigitalInputGain, 0x01:Secondary Mic (Tx), 0x0C:(12 dB)
	0x80150000, //Set Digital Output Gain 0dB
	0x80170000, //0x8017:SetAlgorithmParmID, 0x0000:Suppression Strength
	0x80180004, //0x8018:SetAlgorithmParm, 0x0004:20dB Max Suppression
	0x80170003, //0x8017:SetAlgorithmParmID, 0x0003:AEC Mode
	0x80180000, //0x8018:SetAlgorithmParm, 0x0000:AEC Off
	0x80170004, //0x8017:SetAlgorithmParmID, 0x0004:Use AGC
	0x80180001, //0x8018:SetAlgorithmParm, 0x0001:Yes
	0x80170005, //0x8017:SetAlgorithmParmID, 0x0005:   AGC Target Level (dB)
	0x8018FFF1, //0x8018:SetAlgorithmParm, 0xFFF1:(-15 dB)
	0x80170006, //0x8017:SetAlgorithmParmID, 0x0006:   AGC Noise Floor (dB)
	0x8018FFBF, //0x8018:SetAlgorithmParm, 0xFFBF:(-65 dB)
	0x80170007, //0x8017:SetAlgorithmParmID, 0x0007:   AGC SNR Improve (dB)
	0x80180004, //0x8018:SetAlgorithmParm, 0x0004:(4 dB)
	0x80170026, //0x8017:SetAlgorithmParmID, 0x0026:   AGC Up Rate (dBS)
	0x80180004, //0x8018:SetAlgorithmParm, 0x0004:(4 dBS)
	0x80170027, //0x8017:SetAlgorithmParmID, 0x0027:   AGC Down Rate (dBS)
	0x80180001, //0x8018:SetAlgorithmParm, 0x0001:(1 dBS)
};

unsigned int phonecall_headset[] = {
	0x80260015, // Select audio routing 21
	0x80170002,
	0x80180003, // Select one Mic configuration
	0x800C0400,
	0x800D0004, // Set ADC1 gain to +24dB
	0x801C0000, // Set Voice Processing off
	0x800C0107,
	0x800D0001, // Tri-state PCM0
	0x800C0207,
	0x800D0001, // Tri-state PCM1
};

unsigned int phonecall_speaker[] = {
	0x80260007, // Select audio routing 7
	0x80170002,
	0x80180002, // Select FT Mic configuration
	0x800C0300,
	0x800D0002, // Set ADC0 gain to +12dB
	0x801B000B, // Set Pri Digital input gain to 11dB
	0x80150001, // Set Digital output gain to 1dB
	0x8017001A,
	0x80180000, // Set ComfortNoise off
	0x80170000,
	0x80180004, // set AIS4
	0x801C0001, // Set Voice Processing on
	0x80170004,
	0x80180000, // Use AGC:no
	0x80170020,
	0x80180000, // Tx PostEq Mode 0x0000:Off
	0x800C0107,
	0x800D0001, // Tri-state PCM0
	0x800C0207,
	0x800D0001, // Tri-state PCM1
};

unsigned int phonecall_bt[] = {
	0x80260006, // Select audio routing 6
	0x80170002,
	0x80180003, // Select one Mic configuration
	0x801B0000, // Set Digital input gain to 0dB
	0x80150000, // Set Digital output gain to 0dB
	0x80150100, // Set Downlink digital gain to 0dB
	0x801C0000, // Set Voice Processing off (bypass mode)
	0x800C0107,
	0x800D0000, // set PCM0 TristateEnable disable
	0x800C0207,
	0x800D0000, // set PCM1 TristateEnable disable
};

unsigned int INT_MIC_recording_receiver[] = {
	0x80260007, //Select audio routing 7
	0x80170002,
	0x80180002, //Select FT Mic configuration
	0x800C0300,
	0x800D0001, //Set ADC0 gain to +6dB
	0x801B0009, //Set Pri Digital input gain to 9dB
	0x80150000, //Set Digital output gain to 0dB
	0x8017001A,
	0x80180000, //Set ComfortNoise off
	0x80170000,
	0x80180000, //set AIS4
	0x801C0000, //Set Voice Processing off
	0x80170004,
	0x80180000, //Use AGC:no
	0x80170020,
	0x80180000, //Tx PostEq Mode 0x0000:Off
	0x800C0107,
	0x800D0001, //Tri-state PCM0
	0x800C0207,
	0x800D0001, //Tri-state PCM1
};

unsigned int EXT_MIC_recording[] = {
	0x80260015, // Select audio routing 21
	0x80170002,
	0x80180003, // Select one Mic configuration
	0x800C0400,
	0x800D0002, // Set ADC1 gain to +12dB
	0x801B0009, // Set Pri Digital input gain to 9dB
	0x80150003, // Set Digital output gain to 3dB
	0x8017001A,
	0x80180000, // Set ComfortNoise off
	0x80170000,
	0x80180004, // set AIS4
	0x801C0001, // Set Voice Processing on
	0x80170004,
	0x80180000, // Use AGC:no
	0x80170020,
	0x80180000, // Tx PostEq Mode 0x0000:Off
	0x800C0107,
	0x800D0001, // Tri-state PCM0
	0x800C0207,
	0x800D0001, // Tri-state PCM1
};

unsigned int INT_MIC_recording_speaker[] = {
	0x80260007, // Select audio routing 7
	0x80170002,
	0x80180002, // Select FT Mic configuration
	0x800C0300,
	0x800D0002, // Set ADC0 gain to +12dB
	0x801B000B, // Set Pri Digital input gain to 11dB
	0x80150001, // Set Digital output gain to 1dB
	0x8017001A,
	0x80180000, // Set ComfortNoise off
	0x80170000,
	0x80180004, // set AIS4
	0x801C0001, // Set Voice Processing on
	0x80170004,
	0x80180000, // Use AGC:no
	0x80170020,
	0x80180000, // Tx PostEq Mode 0x0000:Off
	0x800C0107,
	0x800D0001, // Tri-state PCM0
	0x800C0207,
	0x800D0001, // Tri-state PCM1
};

unsigned int BACK_MIC_recording[] = {
	0x80260015, // Select audio routing 21
	0x80170002,
	0x80180002, // Select FT Mic configuration
	0x800C0300,
	0x800D0002, // Set ADC0 gain to +12dB
	0x801B000B, // Set Pri Digital input gain to 11dB
	0x80150001, // Set Digital output gain to 1dB
	0x8017001A,
	0x80180000, // Set ComfortNoise off
	0x80170000,
	0x80180004, // set AIS4
	0x801C0001, // Set Voice Processing on
	0x80170004,
	0x80180000, // Use AGC:no
	0x80170020,
	0x80180000, // Tx PostEq Mode 0x0000:Off
	0x800C0107,
	0x800D0001, // Tri-state PCM0
	0x800C0207,
	0x800D0001, // Tri-state PCM1
};

unsigned int vr_no_ns_receiver[] = {
        0x80260001, // 0x8026:SelectRouting, 0x0001:Pri,Sec,FEi,Lp1 -- CSp,FEo,Lp1
        0x80170002, // 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:Close Talk (CT)
        0x8017001A, // 0x8017:SetAlgorithmParmID, 0x001A:Use ComfortNoise
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No
        0x801C0001, // 0x801C:VoiceProcessingOn, 0x0001:Yes
        0x80170000, // 0x8017:SetAlgorithmParmID, 0x0000:Suppression Strength,
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No suppression
        0x80170004, // 0x8017:SetAlgorithmParmID, 0x0004:Use AGC,
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000: No
        0x800C0300, // 0x800C:SetDeviceParmID, 0x03:ADC0, 0x00:ADC Gain,
        0x800D0001, // 0x800D:SetDeviceParm, 0x0001:+ 6dB
        0x800C0400, // 0x800C:SetDeviceParmID, 0x04:ADC1, 0x00:ADC Gain
        0x800D0001, // 0x800D:SetDeviceParm, 0x0001:+ 6dB
        0x80150000, // 0x8015:SetDigitalOutputGain, 0x00:Tx, 0x00:(0 dB)
        0x801B0009, // 0x801B:SetDigitalInputGain, 0x00:Primay Mic (Tx), 0x09:(9 dB)
        0x801B0109, // 0x801B:SetDigitalInputGain, 0x01:Secondary Mic (Tx), 0x09:(9 dB)
};

unsigned int vr_no_ns_headset[] = {
        0x80170002, // 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration
        0x80180003, // 0x8018:SetAlgorithmParm, 0x0003:1M-DG (1-mic digital input)
        0x80260015, // 0x8026:SelectRouting, 0x0015:Snk,Pri,Snk,Snk - Csp,Zro,Zro (none)
        0x800C0400, // 0x800C:SetDeviceParmID, 0x04:ADC1, 0x00:ADC Gain,
        0x800D0001, // 0x800D:SetDeviceParm, 0x0001:+6 dB
        0x8017001A, // 0x8017:SetAlgorithmParmID, 0x001A:Use ComfortNoise,
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No
        0x801C0001, // 0x801C:VoiceProcessingOn, 0x0001:Yes
        0x801B0012, // Set Digital Input Gain 18dB
        0x80150000, // Set Digital Output Gain 0dB
        0x80170000, // 0x8017:SetAlgorithmParmID, 0x0000:Suppression Strength,
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No Suppression
        0x80170004, // 0x8017:SetAlgorithmParmID, 0x0004:Use AGC,
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No
};

unsigned int vr_no_ns_speaker[] = {
        0x80260007, // 0x8026:SelectRouting, 0x0007:Pri,Snk,Snk,Snk -- CSp,Zro,Zro
        0x80170002, // 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration
        0x80180002, // 0x8018:SetAlgorithmParm, 0x0002:Far Talk (FT)
        0x800C0300, // 0x800C:SetDeviceParmID, 0x03:ADC0, 0x00:ADC Gain
        0x800D0001, // 0x800D:SetDeviceParm, 0x0001:+6dB
        0x800C0400, // 0x800C:SetDeviceParmID, 0x04:ADC1, 0x00:ADC Gain
        0x800D0001, // 0x800D:SetDeviceParm, 0x0001:+6dB
        0x801B000E, // 0x801B:SetDigitalInputGain, 0x0011:(14 dB)
        0x80170000, // 0x8017:SetAlgorithmParmID, 0x0000:Suppression Strength
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No suppression
        0x8017001A, // 0x8017:SetAlgorithmParmID, 0x001A:Use ComfortNoise
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No
        0x801C0001, // 0x801C:VoiceProcessingOn, 0x0001:Yes
        0x80150006, // 0x8015:SetDigitalOutputGain, 0x0006:(6 dB)
};

unsigned int vr_no_ns_bt[] = {
        0x80260006, // Select audio routing 6
        0x801B0000, // Set Digital input gain to 0dB
        0x80150000, // Set Digital output gain to 0dB
        0x80150100, // Set Downlink digital gain to 0dB
        0x801C0000, // Set Voice Processing off (bypass mode)
        0x800C0107,
        0x800D0000, // set PCM0 TristateEnable disable
        0x800C0207,
        0x800D0000, // set PCM1 TristateEnable disable
};

unsigned int vr_ns_receiver[] = {
        0x80260001, // 0x8026:SelectRouting, 0x0001:Pri,Sec,FEi,Lp1 -- CSp,FEo,Lp1
        0x80170002, // 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:Close Talk (CT)
        0x8017001A, // 0x8017:SetAlgorithmParmID, 0x001A:Use ComfortNoise
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No
        0x801C0001, // 0x801C:VoiceProcessingOn, 0x0001:Yes
        0x80170000,
        0x80180004, // Noise Suppression AIS 4
        0x80170004, // 0x8017:SetAlgorithmParmID, 0x0004:Use AGC
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000: No
        0x800C0300, // 0x800C:SetDeviceParmID, 0x03:ADC0, 0x00:ADC Gain,
        0x800D0001, // 0x800D:SetDeviceParm, 0x0001:+ 6dB
        0x800C0400, // 0x800C:SetDeviceParmID, 0x04:ADC1, 0x00:ADC Gain
        0x800D0001, // 0x800D:SetDeviceParm, 0x0000:+ 6dB
        0x80150000, // 0x8015:SetDigitalOutputGain, 0x00:Tx, 0x00:(0 dB)
        0x801B0009, // 0x801B:SetDigitalInputGain, 0x00:Primay Mic (Tx), 0x09:(9 dB)
        0x801B0109, // 0x801B:SetDigitalInputGain, 0x01:Secondary Mic (Tx), 0x09:(9 dB)
};

unsigned int vr_ns_headset[] = {
        0x80170002, // 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration
        0x80180003, // 0x8018:SetAlgorithmParm, 0x0003:1M-DG (1-mic digital input)
        0x80260015, // 0x8026:SelectRouting, 0x0015:Snk,Pri,Snk,Snk - Csp,Zro,Zro (none)
        0x800C0400, // 0x800C:SetDeviceParmID, 0x04:ADC1, 0x00:ADC Gain
        0x800D0001, // 0x800D:SetDeviceParm, 0x0001:+6 dB
        0x8017001A, // 0x8017:SetAlgorithmParmID, 0x001A:Use ComfortNoise,
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No
        0x801C0001, // 0x801C:VoiceProcessingOn, 0x0001:Yes
        0x801B0012, // Set Digital Input Gain 18dB
        0x80150000, // Set Digital Output Gain 0dB
        0x80170000,
        0x80180002, // Noise Suppression AIS2
        0x80170004, // 0x8017:SetAlgorithmParmID, 0x0004:Use AGC
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No
};

unsigned int vr_ns_speaker[] = {
        0x80260007, // 0x8026:SelectRouting, 0x0007:Pri,Snk,Snk,Snk -- CSp,Zro,Zro
        0x80170002, // 0x8017:SetAlgorithmParmID, 0x0002:Microphone Configuration
        0x80180002, // 0x8018:SetAlgorithmParm, 0x0002:Far Talk (FT)
        0x800C0300, // 0x800C:SetDeviceParmID, 0x03:ADC0, 0x00:ADC Gain,
        0x800D0001, // 0x800D:SetDeviceParm, 0x0001:+6dB
        0x800C0400, // 0x800C:SetDeviceParmID, 0x04:ADC1, 0x00:ADC Gain
        0x800D0001, // 0x800D:SetDeviceParm, 0x0001:+6dB
        0x801B000E, // 0x801B:SetDigitalInputGain, 0x0011:(14 dB)
        0x80170000,
        0x80180004, // Noise Suppression AIS4
        0x8017001A, // 0x8017:SetAlgorithmParmID, 0x001A:Use ComfortNoise,
        0x80180000, // 0x8018:SetAlgorithmParm, 0x0000:No
        0x801C0001, // 0x801C:VoiceProcessingOn, 0x0001:Yes
        0x80150006, // 0x8015:SetDigitalOutputGain, 0x0009:(6 dB)
};

unsigned int vr_ns_bt[] = {
        0x80260006, // Select audio routing 6
        0x801B0000, // Set Digital input gain to 0dB
        0x80150000, // Set Digital output gain to 0dB
        0x80150100, // Set Downlink digital gain to 0dB
        0x801C0000, // Set Voice Processing off (bypass mode)
        0x800C0107,
        0x800D0000, // set PCM0 TristateEnable disable
        0x800C0207,
        0x800D0000, // set PCM1 TristateEnable disable
};

unsigned int suspend_mode[] = {
	A100_msg_Sleep,
};

static ssize_t chk_wakeup_a1026(void)
{
	int rc = 0, retry = 3;

	if (A1026_Suspended == 1) {
		/* Enable A1026 clock */
		if (control_a1026_clk) {
			gpio_set_value(pdata->gpio_a1026_clk, 1);
			mdelay(1);
		}
		gpio_set_value(pdata->gpio_a1026_wakeup, 0);
		mdelay(10);
		gpio_set_value(pdata->gpio_a1026_wakeup, 1);
		mdelay(120);

		do {
			rc = execute_cmdmsg(0x80000000); /* issue a Sync CMD */
		} while ((rc < 0) && --retry);

		if (rc < 0) {
			pr_err("A1026 wakeup failed!\n");
			goto wakeup_sync_err;
		}

		A1026_Suspended = 0;
	}
wakeup_sync_err:
	return rc;
}

/* Filter commands according to noise suppression state forced by
 * A1026_SET_NS_STATE ioctl.
 * For this function to operate properly, all configurations must include
 * both A100_msg_Bypass and Mic_Config commands even if default values
 * are selected or if Mic_Config is useless because VP is off */
int a1026_filter_vp_cmd(int cmd, int mode)
{
	int msg = (cmd >> 16) & 0xFFFF;
	int filtered_cmd = cmd;

	if (a1026_NS_state == A1026_NS_STATE_AUTO)
		return cmd;

	switch(msg) {
	case A100_msg_Bypass:
		if (a1026_NS_state == A1026_NS_STATE_OFF)
			filtered_cmd = A1026_msg_VP_OFF;
		else
			filtered_cmd = A1026_msg_VP_ON;
		break;
	case A100_msg_SetAlgorithmParmID:
		a1026_param_ID = cmd & 0xFFFF;
		break;
	case A100_msg_SetAlgorithmParm:
		if (a1026_param_ID == Mic_Config) {
			if (a1026_NS_state == A1026_NS_STATE_CT)
				filtered_cmd = (msg << 16);
			else if (a1026_NS_state == A1026_NS_STATE_FT)
				filtered_cmd = (msg << 16) + 0x0002;
		}
		break;
	default:
		if (mode == A1026_CONFIG_VP)
			filtered_cmd = -1;
		break;
	}

	pr_info("A1026 filter cmd: %x filtered = %x, a1026_NS_state %d, mode %d\n",
			cmd, filtered_cmd, a1026_NS_state, mode);

	return filtered_cmd;
}

int a1026_set_config(char newid, int mode)
{
	int i = 0, rc = 0;
	struct cmd_list new_list;
	int cmd;

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
	case A1026_PATH_VR_NO_NS_RECEIVER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = vr_no_ns_receiver;
		new_list.cnt = sizeof(vr_no_ns_receiver)/sizeof(unsigned int);
		break;
	case A1026_PATH_VR_NO_NS_HEADSET:
		gpio_set_value(pdata->gpio_a1026_micsel, 1);
		new_list.p = vr_no_ns_headset;
		new_list.cnt = sizeof(vr_no_ns_headset)/sizeof(unsigned int);
		break;
	case A1026_PATH_VR_NO_NS_SPEAKER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = vr_no_ns_speaker;
		new_list.cnt = sizeof(vr_no_ns_speaker)/sizeof(unsigned int);
		break;
	case A1026_PATH_VR_NO_NS_BT:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = vr_no_ns_bt;
		new_list.cnt = sizeof(vr_no_ns_bt)/sizeof(unsigned int);
		break;
	case A1026_PATH_VR_NS_RECEIVER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = vr_ns_receiver;
		new_list.cnt = sizeof(vr_ns_receiver)/sizeof(unsigned int);
		break;
	case A1026_PATH_VR_NS_HEADSET:
		gpio_set_value(pdata->gpio_a1026_micsel, 1);
		new_list.p = vr_ns_headset;
		new_list.cnt = sizeof(vr_ns_headset)/sizeof(unsigned int);
		break;
	case A1026_PATH_VR_NS_SPEAKER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = vr_ns_speaker;
		new_list.cnt = sizeof(vr_ns_speaker)/sizeof(unsigned int);
		break;
	case A1026_PATH_VR_NS_BT:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = vr_ns_bt;
		new_list.cnt = sizeof(vr_ns_bt)/sizeof(unsigned int);
		break;
	case A1026_PATH_RECORD_RECEIVER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = INT_MIC_recording_receiver;
		new_list.cnt = sizeof(INT_MIC_recording_receiver)/sizeof(unsigned int);
		break;
	case A1026_PATH_RECORD_HEADSET:
		gpio_set_value(pdata->gpio_a1026_micsel, 1);
		new_list.p = EXT_MIC_recording;
		new_list.cnt = sizeof(EXT_MIC_recording)/sizeof(unsigned int);
		break;
	case A1026_PATH_RECORD_SPEAKER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = INT_MIC_recording_speaker;
		new_list.cnt = sizeof(INT_MIC_recording_speaker)/sizeof(unsigned int);
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
	case A1026_PATH_CAMCORDER:
		gpio_set_value(pdata->gpio_a1026_micsel, 0);
		new_list.p = BACK_MIC_recording;
		new_list.cnt = sizeof(BACK_MIC_recording)/sizeof(unsigned int);
		break;
	default:
		pr_err("A1026 set config: Invalid input\n");
		rc = -1;
		goto input_err;
		break;
	}

	a1026_current_config = newid;
	pr_info("A1026 change mode command count = %d\n", new_list.cnt);
	for (i = 0; i < new_list.cnt; i++) {
		pr_info("A1026 cmds: i = %d, *p = 0x%x\n", i, *(new_list.p));
		cmd = a1026_filter_vp_cmd(*(new_list.p), mode);
		if (cmd != -1) {
			rc = execute_cmdmsg(cmd);
			if (rc < 0)
				break;
			if (cmd == A100_msg_Sleep) {
				A1026_Suspended = 1;
				/* Disable A1026 clock */
				mdelay(120);
				if (control_a1026_clk)
					gpio_set_value(pdata->gpio_a1026_clk, 0);
			}
		}
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
	unsigned int sw_reset = 0;

	memset(msgbuf, 0, sizeof(msgbuf));
	memset(chkbuf, 0, sizeof(chkbuf));
	sw_reset = ((A100_msg_Reset << 16) | RESET_IMMEDIATE);

	msgbuf[0] = (msg >> 24) & 0xFF;
	msgbuf[1] = (msg >> 16) & 0xFF;
	msgbuf[2] = (msg >> 8) & 0xFF;
	msgbuf[3] = msg & 0xFF;

	memcpy(chkbuf, msgbuf, 4);

	rc = A1026I2C_TxData(msgbuf, 4);
	if (rc < 0) {
		pr_err("Tx A1026 Exe Cmd error\n");
		A1026_sw_reset(sw_reset);
		return rc;
	}
	/* We don't need to get Ack after sending out a suspend command */
	if (msgbuf[0] == 0x80 && msgbuf[1] == 0x10
		&& msgbuf[2] == 0x00 && msgbuf[3] == 0x01)
		return rc;

	exe_retry = POLLING_RETRY_CNT;
	while (exe_retry--) {
		rc = 0;

		mdelay(20); /* use polling */
		memset(msgbuf, 0, sizeof(msgbuf));
		rc = Read_A1026_Data_Bytes(msgbuf, 4);
		if (rc < 0) {
			pr_err("A1026 Get Ack Error (%d retry cnt left)\n",
				exe_retry);
			continue;
		}

		if (msgbuf[0] == chkbuf[0] && msgbuf[1] == chkbuf[1]
		&& msgbuf[2] == chkbuf[2] && msgbuf[3] == chkbuf[3]) {
			exe_pass = 1;
			break;
		} else if (msgbuf[0] == 0xff && msgbuf[1] == 0xff) {
			pr_err("A1026 Get Illegal cmd \n");
			rc = -ENOEXEC;
			break;
		} else {
			pr_info("A1026 Get Ack not match (%d retry cnt left)\n",
				exe_retry);
#if DEBUG
			pr_info("msgbuf[0] = 0x%x\n", msgbuf[0]);
			pr_info("msgbuf[1] = 0x%x\n", msgbuf[1]);
			pr_info("msgbuf[2] = 0x%x\n", msgbuf[2]);
			pr_info("msgbuf[3] = 0x%x\n", msgbuf[3]);
#endif
			rc = -EBUSY;
		}
	}

	if (!exe_pass) {
		pr_err("A1026 Get Cmd ACK NG (%d)\n", rc);
		A1026_sw_reset(sw_reset);
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
	unsigned int ns_state;

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
		ret = a1026_set_config(pathid, A1026_CONFIG_FULL);
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
	case A1026_SET_NS_STATE:
		if (copy_from_user(&ns_state, argp, sizeof(ns_state)))
			return -EFAULT;
		pr_info("A1026 set noise suppression %d\n", ns_state);
		if (ns_state < 0 || ns_state >= A1026_NS_NUM_STATES)
			return -EINVAL;
		a1026_NS_state = ns_state;
		if (!A1026_Suspended) {
			a1026_set_config(a1026_current_config, A1026_CONFIG_VP);
		}
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

	ret = gpio_request(pdata->gpio_a1026_clk, "a1026");
	if (ret < 0) {
		control_a1026_clk = 0;
		goto chk_gpio_micsel;
	}
	control_a1026_clk = 1;

	ret = gpio_direction_output(pdata->gpio_a1026_clk, 1);
	if (ret < 0) {
		pr_err("A1026: request clk gpio direction failed\n");
		goto err_free_gpio_clk;
	}

chk_gpio_micsel:
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

	if (control_a1026_clk)
		gpio_set_value(pdata->gpio_a1026_clk, 1);
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
err_free_gpio_clk:
	if (control_a1026_clk)
		gpio_free(pdata->gpio_a1026_clk);
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
