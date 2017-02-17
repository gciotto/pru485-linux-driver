/**
 * This driver provides an interface with the PRU-485 serial controller.
 *
 * Author: Gustavo CIOTTO PINTON
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/mutex.h>

#include <linux/platform_device.h>

#define  DEVICE_NAME "pru485"
#define  CLASS_NAME  "pru485"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gustavo Ciotto Pinton");
MODULE_DESCRIPTION("A driver to interface with the BeagleBone's Programmable Realtime Unit.");
MODULE_VERSION("0.1");

static int majorNumber;
static char message[256] = {0};
static short size_of_message;

static struct class* prucharClass  = NULL;
static struct device* prucharDevice = NULL;

/* struct file_operations function prototypes */
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

/* file operations for file /dev/pru485 */
static struct file_operations fops = {
		.open = dev_open,
		.read = dev_read,
		.write = dev_write,
		.release = dev_release,
};

#define DRV_NAME "pruss_uio"

static int pruss485_remove(struct platform_device *dev);
static int pruss485_probe(struct platform_device *pdev);

static struct platform_driver pruss485_driver = {
		.probe = pruss485_probe,
		.remove = pruss485_remove,
		.driver = {
				.name = DRV_NAME,
		},
};

/* mutex protecting read and writing order */
static DEFINE_MUTEX(pruchar_mutex);

static int __init pru_driver_init(void) {

	printk(KERN_INFO "PRU KVM: initializing module.\n");

	mutex_init(&pruchar_mutex);

	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
	if (majorNumber < 0) {

		printk(KERN_ALERT "PRU KVM: failed to register a major number.\n");
		return majorNumber;
	}
	printk(KERN_INFO "PRU KVM: registered correctly with major number %d\n", majorNumber);

	prucharClass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(prucharClass)) {

		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "PRU KVM: failed to register device class.\n");
		return PTR_ERR(prucharClass);
	}
	printk(KERN_INFO "PRU KVM: device class registered correctly\n");

	prucharDevice = device_create(prucharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
	if (IS_ERR(prucharDevice)){

		mutex_destroy(&pruchar_mutex);
		class_destroy(prucharClass);
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "PRU KVM: Failed to create the device\n");
		return PTR_ERR(prucharDevice);
	}

	if (platform_driver_register(&pruss485_driver)) {

		printk(KERN_INFO "PRU KVM: failed to register platform driver.\n");

		mutex_destroy(&pruchar_mutex);

		device_destroy(prucharClass, MKDEV(majorNumber, 0));
		class_unregister(prucharClass);
		class_destroy(prucharClass);
		unregister_chrdev(majorNumber, DEVICE_NAME);

	}

	printk(KERN_INFO "PRU KVM: device class created correctly\n");

	return 0;
}

static void __exit pru_driver_exit(void) {

	mutex_destroy(&pruchar_mutex);

	device_destroy(prucharClass, MKDEV(majorNumber, 0));
	class_unregister(prucharClass);
	class_destroy(prucharClass);
	unregister_chrdev(majorNumber, DEVICE_NAME);
	platform_driver_unregister(&pruss485_driver);
	printk(KERN_INFO "PRU KVM: module closed.\n");

}

static int dev_open(struct inode *inodep, struct file *filep){

	if(!mutex_trylock(&pruchar_mutex)){    /* Try to acquire the mutex */
		/* returns 1 if successful and 0 if there is contention */
		printk(KERN_ALERT "PRU KVM: Device in use by another process");
		return -EBUSY;
	}

	printk(KERN_INFO "PRU KVM: device has been opened.\n");
	return 0;
}

static int dev_release(struct inode *inodep, struct file *filep){

	mutex_unlock(&pruchar_mutex);

	printk(KERN_INFO "PRU KVM: device successfully closed.\n");
	return 0;
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
	int error_count = 0;
	// copy_to_user has the format ( * to, *from, size) and returns 0 on success
	error_count = copy_to_user(buffer, message, size_of_message);

	if (!error_count) {
		printk(KERN_INFO "PRU KVM: Sent %d characters to the user\n", size_of_message);
		return (size_of_message=0);
	}
	else {
		printk(KERN_INFO "PRU KVM: Failed to send %d characters to the user\n", error_count);
		return -EFAULT;
	}
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){

	sprintf(message, "%s(%zu letters)", buffer, len);
	size_of_message = strlen(message);
	printk(KERN_INFO "EBBChar: Received %zu characters from the user\n", len);
	return len;
}


static int pruss485_probe(struct platform_device *pdev) {

	printk(KERN_INFO "PRU KVM: probe function called.\n");

	return 0;
}

static int pruss485_remove(struct platform_device *dev) {

	printk(KERN_INFO "PRU KVM: remove function called.\n");

        return 0;

}

module_init(pru_driver_init);
module_exit(pru_driver_exit);
