/*
 * synaptics_reg_access.c
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * Description: command line register access implimentation using command
 * line args. This file should not be OS dependant and should build and
 * run under any Linux based OS that utilizes the Synaptice rmi dev
 * built into the kernel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define VERSION "1.1"

#define DEFAULT_SENSOR "/sys/class/input/input0"

#define OPEN_FILENAME "open"
#define ADDRESS_FILENAME "address"
#define LENGTH_FILENAME "length"
#define DATA_FILENAME "data"
#define RELEASE_FILENAME "release"

#define MAX_BUFFER_LEN 256
#define MAX_STRING_LEN 256

char mySensor[MAX_STRING_LEN];

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
	printf("Version %s\n", VERSION);
	printf("Usage: %s [-a {address in hex}] [-l {length to read}] [-d {data to write}] [-r] [-w] [-p {sysfs_entry}]\n", name);

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
	struct stat st;
	FILE *fp;

	if (argc == 1) {
		usage(argv[0]);
		exit(EINVAL);
	}

	while (this_arg < argc) {
		if (!strcmp((const char *)argv[this_arg], "-p")) {
			/* path to sensor sysfs entry */
			this_arg++;

			if (stat(argv[this_arg], &st) == 0) {
				strncpy(mySensor, argv[this_arg],
						MAX_STRING_LEN);
			} else {
				printf("ERROR: sysfs entry %s not found\n",
						argv[this_arg]);
				exit(EINVAL);
			}
		} else if (!strcmp((const char *)argv[this_arg], "-r")) {
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

	if (!strlen(mySensor))
		strncpy(mySensor, DEFAULT_SENSOR, MAX_STRING_LEN);

	snprintf(rmidev_open, MAX_STRING_LEN, "%s/rmidev/%s", mySensor,
							OPEN_FILENAME);
	snprintf(rmidev_address, MAX_STRING_LEN, "%s/rmidev/%s", mySensor,
							ADDRESS_FILENAME);
	snprintf(rmidev_length, MAX_STRING_LEN, "%s/rmidev/%s", mySensor,
							LENGTH_FILENAME);
	snprintf(rmidev_data, MAX_STRING_LEN, "%s/rmidev/%s", mySensor,
							DATA_FILENAME);
	snprintf(rmidev_release, MAX_STRING_LEN, "%s/rmidev/%s", mySensor,
							RELEASE_FILENAME);

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
		printf("error: failed to open %s\n", rmidev_address);
		exit(EIO);
	}
	writevaluetofp(fp, address);
	fclose(fp);

	fp = fopen(rmidev_length, "w");
	if (!fp) {
		printf("error: failed to open %s\n", rmidev_length);
		exit(EIO);
	}
	writevaluetofp(fp, length);
	fclose(fp);

	if (read_write == 1) {
		fp = fopen(rmidev_data, "w");
		if (!fp) {
			printf("error: failed to open %s\n", rmidev_data);
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
			printf("error: failed to open %s\n", rmidev_data);
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
		printf("error: failed to open %s\n", rmidev_release);
		exit(EIO);
	}
	writevaluetofp(fp, 1);
	fclose(fp);

	return 0;
}
