/**
 * This driver provides an interface with the PRU-485 serial controller.
 *
 * Author: Gustavo CIOTTO PINTON
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>

#include <linux/fs.h>

#define  DEVICE_NAME "pru485"
#define  CLASS_NAME  "pru485"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gustavo Ciotto Pinton");
MODULE_DESCRIPTION("A driver to interface with the BeagleBone's Programmable Realtime Unit.");
MODULE_VERSION("0.1");


static int __init pru_driver_init(void) {


	return 0;
}

static void __exit pru_driver_exit(void) {


}


module_init(pru_driver_init);
module_exit(pru_driver_exit);
