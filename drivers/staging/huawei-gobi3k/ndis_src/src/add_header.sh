#!/bin/bash

KPATH=/lib/modules/$(uname -r)/kernel/drivers

if [ "$1" == "modules" ]
then
	if [ "$(uname -r)" \< "2.6.25" ]
	then
	#	cp $(pwd)/usbnet.h  $2  
	echo "make mdoules"
	fi
elif [ "$1" == "clean" ]
then
	if [ "$(uname -r)" \< "2.6.22" ]
	then
		if [ -f $KPATH/usb/net/hw_cdc_driver.ko ]
		then 
			rm -f $KPATH/usb/net/hw_cdc_driver.ko
			depmod -a
		fi
	else
		if [ -f $KPATH/net/usb/cdc_encap.ko ]
		then 
			rm -f $KPATH/net/usb/cdc_encap.ko
		fi
		if [ -f $KPATH/net/usb/hw_cdc_driver.ko ]
		then 
			rm -f $KPATH/net/usb/hw_cdc_driver.ko
		fi
		depmod -a
	fi
	
elif [ "$1" == "install" ]
then
	if [ "$(uname -r)" \< "2.6.22" ]
	then
		if [ -f $(pwd)/hw_cdc_driver.ko ]
		then 
			cp $(pwd)/hw_cdc_driver.ko $KPATH/usb/net
			chmod 744 $KPATH/usb/net/hw_cdc_driver.ko
			depmod -a
		fi
	else
		if [ -f $(pwd)/cdc_encap.ko ]
		then 
			cp $(pwd)/cdc_encap.ko $KPATH/net/usb
			chmod 744 $KPATH/net/usb/cdc_encap.ko
		fi
		if [ -f $(pwd)/hw_cdc_driver.ko ]
		then 
			cp $(pwd)/hw_cdc_driver.ko $KPATH/net/usb
			chmod 744 $KPATH/net/usb/hw_cdc_driver.ko
		fi
		depmod -a
	fi
fi


