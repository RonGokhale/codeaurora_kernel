synaptics_reg_access - read/write RMI registers

usage: ./synaptics_reg_access -a {address in hex} -l {length to read} -d 
				{data to write} [-r] [-w] [-p {sysfs_entry}]

Usage examples
- Read 6 bytes from RMI address 0xe9
	./synaptics_reg_access -a 0xe9 -l 6 -r
	
- Write 1 bytes to RMI address 0xff
	./synaptics_reg_access -a 0xff -d 0x01 -w
	
- Change sysfs entry to /sys/class/input/input5
  (default path is /sys/class/input/input0)
	./synaptics_reg_access -a 0xe9 -l 6 -r -p /sys/class/input/input5

Pleae make sure rmi dev is built in in kernel.

/* ----------------------------------------------------------------------------
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
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