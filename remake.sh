#!/bin/bash

KERNEL_DEVICE=uio_pruss
CAPE=PRUserial485
CAPE_ADDR=ADDRserial485

LSMOD=$(lsmod | grep ${KERNEL_DEVICE})

OVERLAY_PATH=/root/PRUserial485/src
OVERLAY_BASH=overlay.sh

if [[ ! -z ${LSMOD=x} ]]; then
	rmmod -w $KERNEL_DEVICE
fi

CAT_SLOTS=$(cat $SLOTS | grep -w $CAPE)
if [[ ! -z ${CAT_SLOTS=x} ]]; then
	SLOT_NUMBER=$(echo ${CAT_SLOTS%%:*} | sed -e 's/\s//g')
	echo -$SLOT_NUMBER
	echo -$SLOT_NUMBER > $SLOTS
fi

CAT_SLOTS=$(cat $SLOTS | grep -w $CAPE_ADDR)
if [[ ! -z ${CAT_SLOTS=x} ]]; then
	SLOT_NUMBER=$(echo ${CAT_SLOTS%%:*} | sed -e 's/\s//g')
	echo -$SLOT_NUMBER
	echo -$SLOT_NUMBER > $SLOTS
fi

make clean
make
insmod uio_pruss.ko

echo ${CAPE} > /sys/devices/bone_capemgr.9/slots
echo ${CAPE_ADDR} > /sys/devices/bone_capemgr.9/slots

