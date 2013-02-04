synaptics_reg_access - read/write RMI registers

usage: ./synaptics_reg_access -a {address in hex} -l {length to read} -d 
				{data to write} [-r] [-w]

Usage examples
- Read 6 bytes from RMI address 0xe9
	./synaptics_reg_access -a 0xe9 -l 6 -r
	
- Write 1 bytes to RMI address 0xff
	./synaptics_reg_access -a 0xff -d 0x01 -w

Pleae make sure CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI4_DEV is enabled in
the defconfig file when compiling the driver.