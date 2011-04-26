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
		if [ -f $KPATH/usb/serial/option.ko ]
		then 
			rm -f $KPATH/usb/serial/option.ko
			depmod -a
		fi
	else
		if [ -f $KPATH/usb/serial/option.ko ]
		then 
			rm -f $KPATH/usb/serial/option.ko
			depmod -a
		fi
	fi
	
elif [ "$1" == "install" ]
then
	if [ "$(uname -r)" \< "2.6.22" ]
	then
		if [ -f $(pwd)/option.ko ]
		then 
			cp $(pwd)/option.ko $KPATH/usb/serial/
			chmod 744 $KPATH/usb/serial/option.ko
			depmod -a
		fi
	else
		if [ -f $(pwd)/option.ko ]
		then 
			cp $(pwd)/option.ko $KPATH/usb/serial/
			chmod 744 $KPATH/usb/serial/option.ko
			depmod -a
		fi
	fi
fi


