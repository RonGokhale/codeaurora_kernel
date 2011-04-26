#!/bin/bash

rmmod -f cdc_ether
rmmod -f usbnet
rmmod -f hw_cdc_driver

make clean
make modules
make install
