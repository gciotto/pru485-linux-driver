# PRUSS Serial 485 Linux Driver

A character device driver to interface with the PRU 485 serial controller. It adapts the UIO driver `ui_pruss.c` and adds the character device features. This driver was developed to run in BeagleBone Black `bone-79`.

### Building

A `Makefile` is provided in this repository. Just type in `make` to compile `uio_pruss.ko` and `insmod` to add it into the kernel.

### Testing

`uio_pruss_test.c` is an user-space test application and `remake.sh` helps to compile everything and to apply needed capes.
