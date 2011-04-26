#!/bin/bash

rmmod -f cdc_ether
rmmod -f usbnet
modprobe -r hw_cdc_driver

make clean
make modules
make install
