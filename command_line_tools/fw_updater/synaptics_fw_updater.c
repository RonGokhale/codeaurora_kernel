/*
 * synaptics_fw_updater.c
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * Description: command line reflash implimentation using command
 * line args. This file should not be OS dependant and should build and
 * run under any Linux based OS that utilizes the Synaptice firmware
 * update built into the kernel
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
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#define VERSION "2.0"

#define INPUT_PATH "/sys/class/input/input"
#define NUMBER_OF_INPUTS_TO_SCAN 20

#define MAX_STRING_LEN 256
#define MAX_BUFFER_LEN 256

#define DATA_FILENAME "data"
#define IMAGENAME_FILENAME "imagename"
#define IMAGESIZE_FILENAME "imagesize"
#define DOREFLASH_FILENAME "doreflash"
#define CONFIGAREA_FILENAME "configarea"
#define READCONFIG_FILENAME "readconfig"
#define WRITECONFIG_FILENAME "writeconfig"
#define BLOCKSIZE_FILENAME "blocksize"
#define IMAGEBLOCKCOUNT_FILENAME "fwblockcount"
#define CONFIGBLOCKCOUNT_FILENAME "configblockcount"
#define PMCONFIGBLOCKCOUNT_FILENAME "permconfigblockcount"
#define BUILDID_FILENAME "buildid"
#define FLASHPROG_FILENAME "flashprog"
#define DETECT_FILENAME "buildid"
#define WRITELOCKDOWN_FILENAME "writelockdown"

#define UI_CONFIG_AREA 0
#define PERM_CONFIG_AREA 1
#define BL_CONFIG_AREA 2
#define DISP_CONFIG_AREA 3

#define FW_IMAGE_OFFSET 0x100
#define IMAGE_FILE_CHECKSUM_SIZE 4

unsigned char *image_buf = NULL;
unsigned char *config_buf = NULL;
int fileSize;
int lockdown = 0;
int writeLockdown = 0;
int readConfig = 0;
int writeConfig = 0;
int uiConfig = 0;
int pmConfig = 0;
int blConfig = 0;
int dpConfig = 0;
int force = 0;
int verbose = 0;

char mySensor[MAX_STRING_LEN];
char imageFileName[MAX_STRING_LEN];
char input_detect[MAX_STRING_LEN];

enum update_mode {
	NORMAL = 1,
	FORCE = 2,
	LOCKDOWN = 8,
};

static void usage(char *name)
{
	printf("Version %s\n", VERSION);
	printf("Usage: %s [-b {image_file}] [-ld] [-wld] [-r] [-ui] [-pm] [-bl] [-dp] [-f] [-v]\n", name);
	printf("\t[-b {image_file}] - Name of image file\n");
	printf("\t[-ld] - Do lockdown during flash process\n");
	printf("\t[-wld] - Write lockdown block\n");
	printf("\t[-r] - Read config area\n");
	printf("\t[-ui] - UI config area\n");
	printf("\t[-pm] - Permanent config area\n");
	printf("\t[-bl] - BL config area\n");
	printf("\t[-dp] - Display config area\n");
	printf("\t[-f] - Force reflash\n");
	printf("\t[-v] - Verbose output\n");

	return;
}

static void error_exit(error_code)
{
	if (image_buf)
		free(image_buf);

	if (config_buf)
		free(config_buf);

	exit(-error_code);

	return;
}

static void ConvertToLittleEndian(unsigned char *dest, unsigned long src)
{
	dest[0] = (unsigned char)(src & 0xff);
	dest[1] = (unsigned char)((src >> 8) & 0xff);
	dest[2] = (unsigned char)((src >> 16) & 0xff);
	dest[3] = (unsigned char)((src >> 24) & 0xff);

	return;
}

static void TimeSubtract(struct timeval *result, struct timeval *x, struct timeval *y)
{
	int num_of_secs;

	if (x->tv_usec < y->tv_usec) {
		num_of_secs = (y->tv_usec - x->tv_usec) / 1000000 + 1;
		y->tv_usec -= 1000000 * num_of_secs;
		y->tv_sec += num_of_secs;
	}

	if (x->tv_usec - y->tv_usec > 1000000) {
		num_of_secs = (x->tv_usec - y->tv_usec) / 1000000;
		y->tv_usec += 1000000 * num_of_secs;
		y->tv_sec -= num_of_secs;
	}

	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_usec = x->tv_usec - y->tv_usec;

	return;
}

static void WriteBinaryData(char *buf, int len)
{
	int numBytesWritten;
	char tmpfname[MAX_STRING_LEN];
	FILE *fp;

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, DATA_FILENAME);

	fp = fopen(tmpfname, "w");
	if (!fp) {
		printf("ERROR: failed to open %s for writing\n", tmpfname);
		error_exit(EINVAL);
	}

	numBytesWritten = fwrite(buf, 1, len, fp);
	if (numBytesWritten != len) {
		printf("ERROR: failed to write all data to %s\n", tmpfname);
		fclose(fp);
		error_exit(EIO);
	}

	fclose(fp);

	return;
}

static void ReadBinaryData(char *buf, int len)
{
	int numBytesRead;
	char tmpfname[MAX_STRING_LEN];
	FILE *fp;

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, DATA_FILENAME);

	fp = fopen(tmpfname, "r");
	if (!fp) {
		printf("ERROR: failed to open %s for reading\n", tmpfname);
		error_exit(EINVAL);
	}

	numBytesRead = fread(buf, 1, len, fp);
	if (numBytesRead != len) {
		printf("ERROR: failed to read all data from %s\n", tmpfname);
		fclose(fp);
		error_exit(EIO);
	}

	fclose(fp);

	return;
}

static void WriteValueToSysfsFile(char *fname, unsigned int value)
{
	int numBytesWritten;
	char buf[MAX_BUFFER_LEN];
	int fd;

	fd = open(fname, O_WRONLY);
	if (fd < 0) {
		printf("ERROR: failed to open %s for writing\n", fname);
		error_exit(EINVAL);
	}

	snprintf(buf, MAX_BUFFER_LEN, "%u", value);

	numBytesWritten = write(fd, buf, strlen(buf) + 1);
	if (numBytesWritten != ((int)(strlen(buf) + 1))) {
		printf("ERROR: failed to write to %s\n", fname);
		close(fd);
		error_exit(EIO);
	}

	close(fd);

	return;
}

static void ReadValueFromSysfsFile(char *fname, unsigned int *value)
{
	int numBytesRead;
	char buf[MAX_BUFFER_LEN];
	int fd;

	fd = open(fname, O_RDONLY);
	if (fd < 0) {
		printf("ERROR: failed to open %s for reading\n", fname);
		error_exit(EINVAL);
	}

	numBytesRead = read(fd, buf, sizeof(buf));
	if (numBytesRead == -1) {
		printf("ERROR: failed to read all data from %s\n", fname);
		close(fd);
		error_exit(EIO);
	}

	*value = strtoul(buf, NULL, 0);

	close(fd);

	return;
}

static int SetImageName(void)
{
	int numBytesWritten;
	char tmpfname[MAX_STRING_LEN];
	int fd;

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, IMAGENAME_FILENAME);

	fd = open(tmpfname, O_WRONLY);
	if (fd < 0)
		return 0;

	numBytesWritten = write(fd, imageFileName, strlen(imageFileName) + 1);
	if (numBytesWritten != ((int)(strlen(imageFileName) + 1))) {
		printf("ERROR: failed to write to %s\n", tmpfname);
		close(fd);
		error_exit(EIO);
	}

	close(fd);

	return 1;
}

static void SetImageSize(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, IMAGESIZE_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static void StartReflash(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, DOREFLASH_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static void SetConfigArea(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, CONFIGAREA_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static void StartWriteConfig(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, WRITECONFIG_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static void StartWriteLockdown(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, WRITELOCKDOWN_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static void StartReadConfig(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, READCONFIG_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static int ReadBlockSize(void)
{
	unsigned int blockSize;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, BLOCKSIZE_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &blockSize);

	return blockSize;
}

static int ReadFirmwareBlockCount(void)
{
	unsigned int imageBlockCount;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, IMAGEBLOCKCOUNT_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &imageBlockCount);

	return imageBlockCount;
}

static int ReadConfigBlockCount(void)
{
	unsigned int configBlockCount;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, CONFIGBLOCKCOUNT_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &configBlockCount);

	return configBlockCount;
}

static int ReadPmConfigBlockCount(void)
{
	unsigned int configBlockCount;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, PMCONFIGBLOCKCOUNT_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &configBlockCount);

	return configBlockCount;
}

static int ReadBuildID(void)
{
	unsigned int buildID;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, BUILDID_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &buildID);

	return buildID;
}

static int ReadFlashProg(void)
{
	unsigned int flashProg;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, FLASHPROG_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &flashProg);

	return flashProg;
}

static void CalculateChecksum(unsigned short *data, unsigned short len, unsigned long *result)
{
	unsigned long temp;
	unsigned long sum1 = 0xffff;
	unsigned long sum2 = 0xffff;

	*result = 0xffffffff;

	while (len--) {
		temp = *data;
		sum1 += temp;
		sum2 += sum1;
		sum1 = (sum1 & 0xffff) + (sum1 >> 16);
		sum2 = (sum2 & 0xffff) + (sum2 >> 16);
		data++;
	}

	*result = sum2 << 16 | sum1;

	return;
}

static int CompareChecksum(void)
{
	unsigned long headerChecksum;
	unsigned long computedChecksum;

	headerChecksum = (unsigned long)image_buf[0] +
			(unsigned long)image_buf[1] * 0x100 +
			(unsigned long)image_buf[2] * 0x10000 +
			(unsigned long)image_buf[3] * 0x1000000;

	CalculateChecksum((unsigned short *)&image_buf[IMAGE_FILE_CHECKSUM_SIZE],
			((fileSize - IMAGE_FILE_CHECKSUM_SIZE) / 2), &computedChecksum);

	if (verbose) {
		printf("Checksum in image file header = 0x%08x\n", (unsigned int)headerChecksum);
		printf("Checksum computed from image file = 0x%08x\n", (unsigned int)computedChecksum);
	}

	if (headerChecksum == computedChecksum)
		return 1;
	else
		return 0;
}

static int ProceedWithReflash(void)
{
	int index = 0;
	int deviceBuildID;
	int imageBuildID;
	char imagePR[MAX_STRING_LEN];
	char *strptr;

	if (force) {
		printf("Force reflash\n");
		return 1;
	}

	if (ReadFlashProg()) {
		printf("Force reflash (device in flash prog mode)\n");
		return 1;
	}

	strptr = strstr(imageFileName, "PR");
	if (!strptr) {
		printf("No valid PR number (PRxxxxxxx) found in image file name (%s)\n", imageFileName);
		return 0;
	}

	strptr += 2;
	while (strptr[index] >= '0' && strptr[index] <= '9') {
		imagePR[index] = strptr[index];
		index++;
	}
	imagePR[index] = 0;

	imageBuildID = strtoul(imagePR, NULL, 0);
	deviceBuildID = ReadBuildID();
	printf("Device firmware ID = %d\n", deviceBuildID);
	printf("Image firmware ID = %d\n", imageBuildID);

	if (imageBuildID > deviceBuildID) {
		printf("Proceed with reflash\n");
		return 1;
	} else {
		printf("No need to do reflash\n");
		return 0;
	}
}

static void DoReadConfig(void)
{
	int ii;
	int jj;
	int index = 0;
	int firmwareSize;
	int configSize;
	int blockCount;
	int blockSize = ReadBlockSize();
	unsigned long checksum;
	unsigned char checksumArray[IMAGE_FILE_CHECKSUM_SIZE];
	FILE *fp;

	if (uiConfig) {
		SetConfigArea(UI_CONFIG_AREA);
		blockCount = ReadConfigBlockCount();
		configSize = blockSize * blockCount;
	} else if (pmConfig) {
		SetConfigArea(PERM_CONFIG_AREA);
		blockCount = ReadPmConfigBlockCount();
		configSize = blockSize * blockCount;
	} else {
		return;
	}

	StartReadConfig(1);
	config_buf = malloc(configSize);
	if (!config_buf) {
		printf("ERROR: failed to allocate memory for config buffer\n");
		error_exit(ENOMEM);
	}
	ReadBinaryData((char *)&config_buf[0], configSize);

	for (ii = 0; ii < blockCount; ii++) {
		for (jj = 0; jj < blockSize; jj++) {
			printf("0x%02x ", config_buf[index]);
			index++;
		}
		printf("\n");
	}

	if (uiConfig && strlen(imageFileName)) {
		blockCount = ReadFirmwareBlockCount();
		firmwareSize = blockSize * blockCount;

		printf("Saving config data to %s (from 0x%02x for %d bytes)\n",
				imageFileName,
				FW_IMAGE_OFFSET + firmwareSize,
				configSize);

		fp = fopen(imageFileName, "r+b");
		fseek(fp, FW_IMAGE_OFFSET + firmwareSize, SEEK_SET);
		fwrite(config_buf, 1, configSize, fp);

		/* Update checksum in file*/
		fseek(fp, 0, SEEK_SET);
		fread(image_buf, 1, fileSize, fp);
		CalculateChecksum((unsigned short *)&image_buf[IMAGE_FILE_CHECKSUM_SIZE],
				((fileSize - IMAGE_FILE_CHECKSUM_SIZE) / 2),
				&checksum);

		ConvertToLittleEndian(checksumArray, checksum);

		fseek(fp, 0, SEEK_SET);
		fwrite(checksumArray, 1, IMAGE_FILE_CHECKSUM_SIZE, fp);

		fclose(fp);
	}

	free(config_buf);
	config_buf = NULL;

	return;
}

static void DoWriteConfig(void)
{
	printf("Starting config update...\n");

	if (uiConfig)
		SetConfigArea(UI_CONFIG_AREA);
	else if (pmConfig)
		SetConfigArea(PERM_CONFIG_AREA);
	else if (blConfig)
		SetConfigArea(BL_CONFIG_AREA);
	else if (dpConfig)
		SetConfigArea(DISP_CONFIG_AREA);
	else
		return;

	SetImageSize(fileSize);
	WriteBinaryData((char *)&image_buf[0], fileSize);
	StartWriteConfig(1);

	printf("Config update finished...\n");

	return;
}

static void DoWriteLockdown(void)
{
	printf("Starting device lockdown...\n");

	SetImageSize(fileSize);
	WriteBinaryData((char *)&image_buf[0], fileSize);
	StartWriteLockdown(1);

	printf("Device lockdown finished...\n");

	return;
}

static void DoReflash(void)
{
	int update_mode = NORMAL;

	printf("Starting firmware update...\n");

	if (SetImageName()) {
		if (force)
			update_mode = FORCE;

		if (lockdown)
			update_mode |= LOCKDOWN;
	} else if (!ProceedWithReflash()) {
		goto exit;
	}

	SetImageSize(fileSize);
	WriteBinaryData((char *)&image_buf[0], fileSize);
	StartReflash(update_mode);

exit:
	printf("Firmware update finished...\n");

	return;
}

static void InitImageFile(void)
{
	int numBytesRead;
	FILE *fp;

	if (strlen(imageFileName)) {
		fp = fopen(imageFileName, "r");
		if (!fp) {
			printf("ERROR: image file %s not found\n", imageFileName);
			error_exit(EINVAL);
		}

		fseek(fp, 0L, SEEK_END);
		fileSize = ftell(fp);
		if (fileSize == -1) {
			printf("ERROR: failed to determine size of %s\n", imageFileName);
			fclose(fp);
			error_exit(EINVAL);
		}

		fseek(fp, 0L, SEEK_SET);
		image_buf = malloc(fileSize + 1);
		if (!image_buf) {
			printf("ERROR: failed to allocate memory for image buffer\n");
			fclose(fp);
			error_exit(ENOMEM);
		} else {
			numBytesRead = fread(image_buf, 1, fileSize, fp);
			if (numBytesRead != fileSize) {
				printf("ERROR: failed to read entire content of image file\n");
				fclose(fp);
				error_exit(EINVAL);
			}
		}

		fclose(fp);

		if (!(pmConfig || blConfig || dpConfig || uiConfig)) {
			if (!CompareChecksum()) {
				printf("ERROR: failed to validate checksum of image file\n");
				error_exit(EINVAL);
			}
		}
	}

	return;
}

int main(int argc, char* argv[])
{
	int retval;
	int this_arg = 1;
	int found = 0;
	unsigned char temp;
	struct timeval start_time;
	struct timeval end_time;
	struct timeval elapsed_time;
	struct stat st;
	int fd;

	if (argc == 1) {
		usage(argv[0]);
		error_exit(EINVAL);
	}

	for (temp = 0; temp < NUMBER_OF_INPUTS_TO_SCAN; temp++) {
		memset(input_detect, 0x00, MAX_STRING_LEN);
		snprintf(input_detect, MAX_STRING_LEN, "%s%d/%s", INPUT_PATH,
				(unsigned int)temp, DETECT_FILENAME);
		retval = stat(input_detect, &st);
		if (retval == 0) {
			snprintf(mySensor, MAX_STRING_LEN, "%s%d", INPUT_PATH,
					(unsigned int)temp);
			found = 1;
			break;
		}
	}

	if (!found) {
		printf("ERROR: input driver not found\n");
		error_exit(ENODEV);
	}

	while (this_arg < argc) {
		if (!strcmp((const char *)argv[this_arg], "-b")) {
			/* Image file */
			this_arg++;
			if (this_arg >= argc) {
				printf("ERROR: image file missing\n");
				error_exit(EINVAL);
			}
			/* Check for presence of image file */
			fd = open(argv[this_arg], O_RDONLY);
			if (fd < 0) {
				printf("ERROR: image file %s not found\n", argv[this_arg]);
				error_exit(EINVAL);
			}
			close(fd);
			strncpy(imageFileName, argv[this_arg], MAX_STRING_LEN);
		} else if (!strcmp((const char *)argv[this_arg], "-ld")) {
			lockdown = 1;
		} else if (!strcmp((const char *)argv[this_arg], "-wld")) {
			writeLockdown = 1;	
		} else if (!strcmp((const char *)argv[this_arg], "-r")) {
			readConfig = 1;
		} else if (!strcmp((const char *)argv[this_arg], "-ui")) {
			uiConfig = 1;
		} else if (!strcmp((const char *)argv[this_arg], "-pm")) {
			pmConfig = 1;
		} else if (!strcmp((const char *)argv[this_arg], "-bl")) {
			blConfig = 1;
		} else if (!strcmp((const char *)argv[this_arg], "-dp")) {
			dpConfig = 1;
		} else if (!strcmp((const char *)argv[this_arg], "-f")) {
			force = 1;
		} else if (!strcmp((const char *)argv[this_arg], "-v")) {
			verbose = 1;
		} else {
			usage(argv[0]);
			printf("ERROR: invalid parameter %s\n", argv[this_arg]);
			error_exit(EINVAL);
		}
		this_arg++;
	}

	if ((uiConfig + pmConfig + blConfig + dpConfig + writeLockdown) > 1) {
		printf("ERROR: too many parameters\n");
		error_exit(EINVAL);
	}

	if (uiConfig || pmConfig || blConfig || dpConfig)
		writeConfig = 1;

	if (!readConfig && !strlen(imageFileName)) {
		printf("ERROR: no input file specified\n");
		error_exit(EINVAL);
	}

	InitImageFile();

	retval = gettimeofday(&start_time, NULL);
	if (retval < 0)
		printf("WARNING: failed to get start time\n");

	if (verbose) {
		printf("Sensor sysfs entry: %s\n", mySensor);
		if (!readConfig)
			printf("Image file: %s\n", imageFileName);
	}

	if (readConfig)
		DoReadConfig();
	else if (writeConfig)
		DoWriteConfig();
	else if (writeLockdown)
		DoWriteLockdown();
	else	
		DoReflash();

	retval = gettimeofday(&end_time, NULL);
	if (retval < 0)
		printf("WARNING: failed to get end time\n");

	TimeSubtract(&elapsed_time, &end_time, &start_time);

	if (verbose) {
		printf("Elapsed time = %ld.%06ld seconds\n",
				(long)elapsed_time.tv_sec,
				(long)elapsed_time.tv_usec);
	}

	if (image_buf)
		free(image_buf);

	if (config_buf)
		free(config_buf);

	return 0;
}
