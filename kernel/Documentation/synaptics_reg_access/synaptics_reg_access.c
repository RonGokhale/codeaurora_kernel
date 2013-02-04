/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Copyright © 2011, 2012 Synaptics Incorporated. All rights reserved.
 *
 * The information in this file is confidential under the terms
 * of a non-disclosure agreement with Synaptics and is provided
 * AS IS without warranties or guarantees of any kind.
 *
 * The information in this file shall remain the exclusive property
 * of Synaptics and may be the subject of Synaptics patents, in
 * whole or part. Synaptics intellectual property rights in the
 * information in this file are not expressly or implicitly licensed
 * or otherwise transferred to you as a result of such information
 * being made available to you.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define OPEN_PATH "/sys/class/input/input1/rmidev/open"
#define ADDRESS_PATH "/sys/class/input/input1/rmidev/address"
#define LENGTH_PATH "/sys/class/input/input1/rmidev/length"
#define DATA_PATH "/sys/class/input/input1/rmidev/data"
#define RELEASE_PATH "/sys/class/input/input1/rmidev/release"

#define MAX_BUFFER_LEN 256
#define MAX_STRING_LEN 256

char rmidev_open[MAX_STRING_LEN];
char rmidev_address[MAX_STRING_LEN];
char rmidev_length[MAX_STRING_LEN];
char rmidev_data[MAX_STRING_LEN];
char rmidev_release[MAX_STRING_LEN];

unsigned char read_write = 0; /* 0 = read, 1 = write */
unsigned short address = 0;
unsigned int length = 1;

static void usage(char *name)
{
	printf("usage: %s -a {address in hex} -l {length to read} -d {data to write} [-r] [-w]\n", name);

	return;
}

static void writevaluetofp(FILE *fp, unsigned int value)
{
	int numBytesWritten = 0;
	char buf[MAX_BUFFER_LEN];

	snprintf(buf, MAX_BUFFER_LEN, "%u", value);

	fseek(fp, 0, 0);
	numBytesWritten = fwrite(buf, 1, strlen(buf) + 1, fp);
	if (numBytesWritten != ((int)(strlen(buf) + 1))) {
		printf("error: failed to write all bytes to file\n");
		fclose(fp);
		exit(EIO);
	}

	return;
}

static int CheckOneFile(char* filename)
{
	int retval;
	struct stat st;

	retval = stat(filename, &st);

	if (retval)
		printf("error: %s does not appear to exist\n", filename);

	return retval;
}

static int CheckFiles(void)
{
	int retval;

	retval = CheckOneFile(rmidev_open);
	if (retval)
		return retval;

	retval = CheckOneFile(rmidev_address);
	if (retval)
		return retval;

	retval = CheckOneFile(rmidev_length);
	if (retval)
		return retval;

	retval = CheckOneFile(rmidev_data);
	if (retval)
		return retval;

	retval = CheckOneFile(rmidev_release);
	if (retval)
		return retval;

	return 0;
}

int main(int argc, char* argv[])
{
	int retval;
	int this_arg = 1;
	unsigned long temp;
	unsigned char w_data;
	unsigned char r_data[MAX_BUFFER_LEN];
	FILE *fp;

	if (argc == 1) {
		usage(argv[0]);
		exit(EINVAL);
	}

	while (this_arg < argc) {
		if (!strcmp((const char *)argv[this_arg], "-r")) {
			read_write = 0;
		} else if (!strcmp((const char *)argv[this_arg], "-w")) {
			read_write = 1;
		} else if (!strcmp((const char *)argv[this_arg], "-a")) {
			this_arg++;
			temp = strtoul(argv[this_arg], NULL, 0);
			address = (unsigned short)temp;
		} else if (!strcmp((const char *)argv[this_arg], "-l")) {
			this_arg++;
			temp = strtoul(argv[this_arg], NULL, 0);
			length = (unsigned short)temp;
		} else if (!strcmp((const char *)argv[this_arg], "-d")) {
			this_arg++;
			temp = strtoul(argv[this_arg], NULL, 0);
			w_data = (unsigned char)temp;
		} else {
			usage(argv[0]);
			printf("error: invalid parameter %s\n", argv[this_arg]);
			exit(EINVAL);
		}
		this_arg++;
	}
	strncpy(rmidev_open, OPEN_PATH, MAX_STRING_LEN);
	strncpy(rmidev_address, ADDRESS_PATH, MAX_STRING_LEN);
	strncpy(rmidev_length, LENGTH_PATH, MAX_STRING_LEN);
	strncpy(rmidev_data, DATA_PATH, MAX_STRING_LEN);
	strncpy(rmidev_release, RELEASE_PATH, MAX_STRING_LEN);

	if (CheckFiles())
		exit(ENODEV);

	fp = fopen(rmidev_open, "w");
	if (!fp) {
		printf("error: failed to open %s\n", rmidev_open);
		exit(EIO);
	}
	writevaluetofp(fp, 1);
	fclose(fp);

	fp = fopen(rmidev_address, "w");
	if (!fp) {
		printf("error: failed to open %s\n", rmidev_open);
		exit(EIO);
	}
	writevaluetofp(fp, address);
	fclose(fp);

	fp = fopen(rmidev_length, "w");
	if (!fp) {
		printf("error: failed to open %s\n", rmidev_open);
		exit(EIO);
	}
	writevaluetofp(fp, length);
	fclose(fp);

	if (read_write == 1) {
		fp = fopen(rmidev_data, "w");
		if (!fp) {
			printf("error: failed to open %s\n", rmidev_open);
			exit(EIO);
		}
		fseek(fp, 0, 0);
		retval = fwrite(&w_data, 1, 1, fp);
		if (retval != 1) {
			printf("error: failed to write data\n");
			fclose(fp);
			exit(EIO);
		}
		fclose(fp);
	} else {
		fp = fopen(rmidev_data, "r");
		if (!fp) {
			printf("error: failed to open %s\n", rmidev_open);
			exit(EIO);
		}
		fseek(fp, 0, 0);
		retval = fread(r_data, 1, length, fp);
		if (retval != length) {
			printf("error: failed to read data\n");
			fclose(fp);
			exit(EIO);
		}
		fclose(fp);

		for(temp = 0; temp < length; temp++) {
			printf("data %d = 0x%02x\n", (unsigned int)temp, r_data[temp]);
		}
	}

	fp = fopen(rmidev_release, "w");
	if (!fp) {
		printf("error: failed to open %s\n", rmidev_open);
		exit(EIO);
	}
	writevaluetofp(fp, 1);
	fclose(fp);

	return 0;
}
