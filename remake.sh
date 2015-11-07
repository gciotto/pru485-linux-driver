#!/bin/bash

KERNEL_DEVICE=uio_pruss
CAPE=PRUserial485

LSMOD=$(lsmod | grep ${KERNEL_DEVICE})

if [ ! -z ${LSMOD+x} ]; then
	rmmod ${KERNEL_DEVICE}
fi

CAT_SLOTS=$(cat $SLOTS | grep -w $CAPE)
if [ ! -z ${CAT_SLOTS+x} ]; then
	SLOT_NUMBER=$(sed -e "s/\([0-9]*\)" $CAT_SLOTS)
	echo ${SLOT_NUMBER}
fi