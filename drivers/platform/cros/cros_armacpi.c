/*
 *  ChromeOS platform support code. Glue layer between higher level functions
 *  and per-platform firmware interfaces.
 *
 *  Copyright (C) 2011 The Chromium OS Authors
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
 */

#include <linux/types.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/setup.h>
#include "cros_acpi.h"

/* TODO:
 * let firmware assign gpio_lines
 * u32 gpio_data = (gpio_lines << 1) | is_active_high;
 */

/* gpio lines */
#define GPIO_RECOVERY	56
#define GPIO_DEVELOPER	168
#define GPIO_FW_WP	59

/* TODO:
 * replace the shared memory block with FDT
 * or structured ATAGS
 */

/* default shared address */
#define FIRMWARE_SHARED_DEFAULT_ADDR 0x3ff00000
#define FIRMWARE_SHARED_PHYSICAL_SIZE (1024 * 1024)

/* shared data layout in phisical memory */
#define SIZE_SIGNATURE		16
#define SIZE_CHSW		4
#define SIZE_HWID		256
#define SIZE_FWID		256
#define SIZE_FRID		256
#define SIZE_BOOT_REASON	4
#define SIZE_ACTIVE_MAIN	4
#define SIZE_ACTIVE_EC		4
#define SIZE_ACTIVE_MAIN_TYPE	4
#define SIZE_RECOVERY_REASON	4
#define SIZE_GPIO		(4 * 11)
#define SIZE_NV_OFFSET		4
#define SIZE_NV_SIZE		4
#define SIZE_FMAP_ADDR		8
#define SIZE_NVBLK_LBA		8
#define SIZE_NV_COPY		16

#define OFFSET_SIGNATURE	0
#define OFFSET_CHSW		(OFFSET_SIGNATURE + SIZE_SIGNATURE)
#define OFFSET_HWID		(OFFSET_CHSW + SIZE_CHSW)
#define OFFSET_FWID		(OFFSET_HWID + SIZE_HWID)
#define OFFSET_FRID		(OFFSET_FWID + SIZE_FWID)
#define OFFSET_BOOT_REASON	(OFFSET_FRID + SIZE_FRID)
#define OFFSET_ACTIVE_MAIN	(OFFSET_BOOT_REASON + SIZE_BOOT_REASON)
#define OFFSET_ACTIVE_EC	(OFFSET_ACTIVE_MAIN + SIZE_ACTIVE_MAIN)
#define OFFSET_ACTIVE_MAIN_TYPE	(OFFSET_ACTIVE_EC + SIZE_ACTIVE_EC)
#define OFFSET_RECOVERY_REASON	(OFFSET_ACTIVE_MAIN_TYPE + SIZE_ACTIVE_MAIN_TYPE)
#define OFFSET_GPIO		(OFFSET_RECOVERY_REASON + SIZE_RECOVERY_REASON)
#define OFFSET_NV_OFFSET	(OFFSET_GPIO + SIZE_GPIO)
#define OFFSET_NV_SIZE		(OFFSET_NV_OFFSET + SIZE_NV_OFFSET)
#define OFFSET_FMAP_ADDR	(OFFSET_NV_SIZE + SIZE_NV_SIZE)
#define OFFSET_NVBLK_LBA	(OFFSET_FMAP_ADDR + SIZE_FMAP_ADDR)
#define OFFSET_NV_COPY		(OFFSET_NVBLK_LBA + SIZE_NVBLK_LBA)

int acpi_disabled = 0;
static unsigned long phys_end;
static void *firmware_shared_data;
static u64 firmware_cookie_lba;

static inline u32 get_firmware_u32(unsigned offset)
{
	if (!firmware_shared_data)
		return 0;
	return (u32)(*(u32*)(firmware_shared_data + offset));
}

static inline u64 get_firmware_u64(unsigned offset)
{
	if (!firmware_shared_data)
		return 0;
	return (u64)(*(u64*)(firmware_shared_data + offset));
}

static inline char *get_firmware_str(unsigned offset)
{
	if (!firmware_shared_data)
		return NULL;
	return (char*)firmware_shared_data + offset;
}

static inline int check_firmware_signature(void)
{
	static const u64 signature = 0x534f454d4f524843ull;
	u64 fw_sig = get_firmware_u64(0);
	if (fw_sig == signature)
		return 0;
	return -1;
}

static union acpi_object *append_package(union acpi_object *obj,
						unsigned count)
{
	union acpi_object *p	= obj;
	p->type			= ACPI_TYPE_PACKAGE;
	p->package.type		= ACPI_TYPE_PACKAGE;
	p->package.count	= count;
	p->package.elements	= p + 1;
	++p;
	return p;
}

static union acpi_object *append_integer(union acpi_object *obj,
						int value)
{
	union acpi_object *p	= obj;
	p->type			= ACPI_TYPE_INTEGER;
	p->integer.type		= ACPI_TYPE_INTEGER;
	p->integer.value	= value;
	++p;
	return p;
}

static union acpi_object *append_string(union acpi_object *obj,
						char *string, unsigned maxlen)
{
	union acpi_object *p	= obj;
	p->type			= ACPI_TYPE_STRING;
	p->string.type		= ACPI_TYPE_STRING;
	p->string.pointer	= string;
	if (string) {
		p->string.length	= strnlen(string, maxlen);
	} else {
		p->string.length	= 0;
	}
	++p;
	return p;
}

static acpi_status return_buffer(struct acpi_buffer *buf,
	union acpi_object *obj, size_t obj_size)
{
	acpi_status ret = 0;
	if (buf->length != (acpi_size)(-1)) {
		if (buf->length < obj_size) {
			ret = -1;
			goto out_free;
		}
		memcpy(buf->pointer, obj, obj_size);
		goto out_free;
	} else {
		buf->length	= obj_size;
		buf->pointer	= obj;
		goto out;
	}
out_free:
	kfree(obj);
out:
	return ret;
}

#define OBJ_SIZE sizeof(union acpi_object)
#define CHSW_SIZE (2 * OBJ_SIZE)
static acpi_status get_chsw(struct acpi_buffer *buf)
{
	union acpi_object *p;
	union acpi_object *obj = kzalloc(CHSW_SIZE, GFP_KERNEL);
	if (!obj)
		return -1;
	p = append_package(obj, 1);
	p = append_integer(p, get_firmware_u32(OFFSET_CHSW));
	return return_buffer(buf, obj, CHSW_SIZE);
}

#define HWID_SIZE ((2 * OBJ_SIZE) + 256)
static acpi_status get_hwid(struct acpi_buffer *buf)
{
	union acpi_object *p;
	union acpi_object *obj = kzalloc(HWID_SIZE, GFP_KERNEL);
	if (!obj)
		return -1;
	p = append_package(obj, 1);
	p = append_string(p, get_firmware_str(OFFSET_HWID), 255);
	return return_buffer(buf, obj, HWID_SIZE);
}

#define FWID_SIZE ((2 * OBJ_SIZE) + 256)
static acpi_status get_fwid(struct acpi_buffer *buf)
{
	union acpi_object *p;
	union acpi_object *obj = kzalloc(FWID_SIZE, GFP_KERNEL);
	if (!obj)
		return -1;
	p = append_package(obj, 1);
	p = append_string(p, get_firmware_str(OFFSET_FWID), 255);
	return return_buffer(buf, obj, FWID_SIZE);
}

#define FRID_SIZE ((2 * OBJ_SIZE) + 256)
static acpi_status get_frid(struct acpi_buffer *buf)
{
	union acpi_object *p;
	union acpi_object *obj = kzalloc(FRID_SIZE, GFP_KERNEL);
	if (!obj)
		return -1;
	p = append_package(obj, 1);
	p = append_string(p, get_firmware_str(OFFSET_FRID), 255);
	return return_buffer(buf, obj, FRID_SIZE);
}

#define BINF_SIZE (OBJ_SIZE * 6)
static acpi_status get_binf(struct acpi_buffer *buf)
{
	union acpi_object *p;
	union acpi_object *obj = kzalloc(BINF_SIZE, GFP_KERNEL);
	if (!obj)
		return -1;
	p = append_package(obj, 5);
	p = append_integer(p, get_firmware_u32(OFFSET_BOOT_REASON));
	p = append_integer(p, get_firmware_u32(OFFSET_ACTIVE_MAIN));
	p = append_integer(p, get_firmware_u32(OFFSET_ACTIVE_EC));
	p = append_integer(p, get_firmware_u32(OFFSET_ACTIVE_MAIN_TYPE));
	p = append_integer(p, get_firmware_u32(OFFSET_RECOVERY_REASON));
	return return_buffer(buf, obj, BINF_SIZE);
}

static inline union acpi_object *setup_gpio_obj(union acpi_object *p,
	unsigned signal_type, unsigned gpio_line, unsigned active_high,
	char *chip)
{
	/* emulate x86 nm10 gpio chip */
	int nm10_line = -192;
	nm10_line += (int)gpio_line;
	p = append_integer(p, signal_type);
	p = append_integer(p, active_high);
	p = append_integer(p, nm10_line);
	p = append_string(p, chip, 4);
	return p;
}

/* gpios
 * 1 - recovery
 * 2 - developer switch
 * 3 - firmware write protect
 */
#define N_GPIO 3 /* switches */
#define GPIO_SIZE ((OBJ_SIZE * (1 + (N_GPIO * 5))) + 5) /* "NM10\0" */
static acpi_status get_gpio(struct acpi_buffer *buf)
{
	static const unsigned signal_types[] = {
		0x00000001, 0x00000002, 0x00000003};
	static const unsigned gpio_lines[] = {
		GPIO_RECOVERY, GPIO_DEVELOPER, GPIO_FW_WP};
	char *chip;
	union acpi_object *p, *lv1, *lv2;
	union acpi_object *obj = kzalloc(GPIO_SIZE, GFP_KERNEL);
	int i;
	if (!obj)
		return -1;
	/* controller name = "NM10" */
	chip = (char*)obj + GPIO_SIZE - 5;
	strncpy(chip, "NM10", 4);
	/* setup 2 levels of package
	 * [root]
	 *   [p1]               <-- lv1
	 *   [p2]
	 *   [p3]
	 *     [i] [i] [i] [s]  <-- lv2
	 *     [i] [i] [i] [s]
	 *     [i] [i] [i] [s]
	 */
	/* root package  */
	lv1 = append_package(obj, N_GPIO);
	p = lv1;
	/* level 1 packages */
	for (i = 0; i < N_GPIO; i++)
		p = append_package(p, 4);
	lv2 = p;
	/* level 2 data objects */
	for (i = 0; i < N_GPIO; i++) {
		p = setup_gpio_obj(p, signal_types[i], gpio_lines[i],
			get_firmware_u32(OFFSET_GPIO + (i * 4)), chip);
		lv1->package.elements = lv2;
		lv2 = p;
		++lv1;
	}
	return return_buffer(buf, obj, GPIO_SIZE);
}

#define VBNV_SIZE (OBJ_SIZE * 3)
static acpi_status get_vbnv(struct acpi_buffer *buf)
{
	union acpi_object *p;
	union acpi_object *obj = kzalloc(VBNV_SIZE, GFP_KERNEL);
	unsigned nv_offset, nv_size;
	if (!obj)
		return -1;
	p = append_package(obj, 2);
	nv_offset = get_firmware_u32(OFFSET_NV_OFFSET);
	nv_size   = get_firmware_u32(OFFSET_NV_SIZE);
	p = append_integer(p, nv_offset);
	p = append_integer(p, nv_size);
	return return_buffer(buf, obj, VBNV_SIZE);
}

#define FMAP_SIZE (2 * OBJ_SIZE)
static acpi_status get_fmap(struct acpi_buffer *buf)
{
	union acpi_object *p;
	union acpi_object *obj = kzalloc(FMAP_SIZE, GFP_KERNEL);
	if (!obj)
		return -1;
	p = append_package(obj, 1);
	p = append_integer(p, get_firmware_u64(OFFSET_FMAP_ADDR));

	return return_buffer(buf, obj, FMAP_SIZE);
}

#define N_METHOD	8
#define MLST_STR_SIZE	(5 * N_METHOD)
#define MLST_SIZE	(OBJ_SIZE * (1 + N_METHOD) + MLST_STR_SIZE)
static acpi_status get_mlst(struct acpi_buffer *buf)
{
	static const char *methods =
		"CHSW\0FWID\0HWID\0FRID\0BINF\0GPIO\0VBNV\0FMAP";
	char *string_table;
	int i;
	union acpi_object *p;
	union acpi_object *obj = kzalloc(MLST_SIZE, GFP_KERNEL);
	if (!obj)
		return -1;
	/* string table */
	string_table = (char*)obj + MLST_SIZE - MLST_STR_SIZE;
	memcpy(string_table, methods, N_METHOD * 5);
	/* package and objects */
	p = append_package(obj, N_METHOD);
	for (i = 0; i < N_METHOD; i++) {
		p = append_string(p, string_table, 4);
		string_table += 5;
	}
	return return_buffer(buf, obj, MLST_SIZE);
}

acpi_status acpi_evaluate_object(acpi_handle object, char *pathname,
	struct acpi_object_list *parameter_objects,
	struct acpi_buffer *return_object_output)
{
	if (!return_object_output)
		return -1;
	if (parameter_objects)
		return -1;

	printk("arm acpi: acpi_evaluate_object %s\n", pathname);
	if (!strncmp(pathname, "CHSW", 4))
		return get_chsw(return_object_output);
	else if (!strncmp(pathname, "HWID", 4))
		return get_hwid(return_object_output);
	else if (!strncmp(pathname, "FWID", 4))
		return get_fwid(return_object_output);
	else if (!strncmp(pathname, "FRID", 4))
		return get_frid(return_object_output);
	else if (!strncmp(pathname, "BINF", 4))
		return get_binf(return_object_output);
	else if (!strncmp(pathname, "GPIO", 4))
		return get_gpio(return_object_output);
	else if (!strncmp(pathname, "VBNV", 4))
		return get_vbnv(return_object_output);
	else if (!strncmp(pathname, "FMAP", 4))
		return get_fmap(return_object_output);
	else if (!strncmp(pathname, "MLST", 4))
		return get_mlst(return_object_output);
	return -1;
}

/* get_blknv_sector()
 * return the sector stores firmware cookie
 */
sector_t get_blknv_sector(void)
{
	return firmware_cookie_lba;
}
EXPORT_SYMBOL(get_blknv_sector);

/* get_firmware_nvram_addr()
 * return the iomapped address (for driver)
 * to access NVRAM, before block device been
 * initialized
 */
void *get_firmware_nvram_addr(void)
{
	return (firmware_shared_data + OFFSET_NV_COPY);
}
EXPORT_SYMBOL(get_firmware_nvram_addr);

unsigned get_firmware_nvram_size(void)
{
	return get_firmware_u32(OFFSET_NV_SIZE);
}
EXPORT_SYMBOL(get_firmware_nvram_size);

int cros_acpi_init(void)
{
	int iter;
	int ret = -1;
	struct membank *bank = meminfo.bank;

	for_each_bank(iter, &meminfo) {
		if (bank_phys_end(&bank[iter]) > phys_end)
			phys_end = bank_phys_end(&bank[iter]);
	}
	printk(KERN_INFO "arm acpi: phys_end = %08x\n", (unsigned)phys_end);
	if (phys_end && (!firmware_shared_data) && (phys_end < 0x40000000))
		firmware_shared_data = ioremap(phys_end,
			FIRMWARE_SHARED_PHYSICAL_SIZE);
	if (!firmware_shared_data)
		return -1;
	if (check_firmware_signature()) {
		printk(KERN_ERR "arm acpi: signature verification failed!\n");
		ret = -1;
		goto outunmap;
	}
	firmware_cookie_lba = get_firmware_u64(OFFSET_NVBLK_LBA);
	printk("arm acpi: firmware cookie lba = %llx\n", firmware_cookie_lba);
	return 0;
outunmap:
	iounmap(firmware_shared_data);
	firmware_shared_data = NULL;
	return ret;
}
EXPORT_SYMBOL(cros_acpi_init);

void cros_acpi_release(void)
{
	if (firmware_shared_data) {
		iounmap(firmware_shared_data);
		firmware_shared_data = NULL;
	}
}
EXPORT_SYMBOL(cros_acpi_release);

int acpi_bus_register_driver(struct acpi_driver *driver)
{
	struct acpi_device dev;
	if (!(driver && driver->ops.add)) {
		return -1;
	}
	return (*driver->ops.add)(&dev);
}
EXPORT_SYMBOL(acpi_bus_register_driver);

