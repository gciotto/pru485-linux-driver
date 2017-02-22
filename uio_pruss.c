/*
 * Programmable Real-Time Unit Sub System (PRUSS) UIO driver (uio_pruss)
 *
 * This driver exports PRUSS host event out interrupts and PRUSS, L3 RAM,
 * and DDR RAM to user space for applications interacting with PRUSS firmware
 *
 * Copyright (C) 2010-11 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/uio_driver.h>
#include <linux/platform_data/uio_pruss.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>
#include <linux/pm_runtime.h>

#define DRV_NAME "pruss_uio"
#define DRV_VERSION "1.0"

static int sram_pool_sz = SZ_16K;
module_param(sram_pool_sz, int, 0);
MODULE_PARM_DESC(sram_pool_sz, "sram pool size to allocate ");

static int extram_pool_sz = SZ_256K;
module_param(extram_pool_sz, int, 0);
MODULE_PARM_DESC(extram_pool_sz, "external ram pool size to allocate");

/*
 * Host event IRQ numbers from PRUSS - PRUSS can generate up to 8 interrupt
 * events to AINTC of ARM host processor - which can be used for IPC b/w PRUSS
 * firmware and user space application, async notification from PRU firmware
 * to user space application
 * 3	PRU_EVTOUT0
 * 4	PRU_EVTOUT1
 * 5	PRU_EVTOUT2
 * 6	PRU_EVTOUT3
 * 7	PRU_EVTOUT4
 * 8	PRU_EVTOUT5
 * 9	PRU_EVTOUT6
 * 10	PRU_EVTOUT7
 */
#define MAX_PRUSS_EVT	8

#define PINTC_HIDISR	0x0038
#define PINTC_HIPIR		0x0900
#define HIPIR_NOPEND	0x80000000
#define PINTC_HIER		0x1500

struct uio_pruss_dev {
	struct uio_info *info;
	struct clk *pruss_clk;
	dma_addr_t sram_paddr;
	dma_addr_t ddr_paddr;
	void __iomem *prussio_vaddr;
	unsigned long sram_vaddr;
	void *ddr_vaddr;
	unsigned int hostirq_start;
	unsigned int pintc_base;
	struct gen_pool *sram_pool;
};

#ifdef PRUSS_CHAR_DEVICE
/* Interruption ID */
#define PRU_EVTOUT 	 3
/* Saves platform_device to be used by the character device */
static struct platform_device* _pdev;
static int _pdev_c;
/* Completion variable to signal an interruption */
static DECLARE_COMPLETION(intr_completion);
#endif

static ssize_t store_sync_ddr(struct device *dev, struct device_attribute *attr,  char *buf, size_t count) {
	struct uio_pruss_dev *gdev;
	gdev = dev_get_drvdata(dev);
	dma_sync_single_for_cpu(dev, gdev->ddr_paddr, extram_pool_sz, DMA_FROM_DEVICE);
	return count;
}
static DEVICE_ATTR(sync_ddr, S_IWUSR, NULL, store_sync_ddr);

static const struct attribute *uio_sysfs_attrs[] = {
		&dev_attr_sync_ddr.attr,
		NULL
};

static int uio_sysfs_init(struct platform_device *pdev) {
	int error;
	error = sysfs_create_files(&pdev->dev.kobj, uio_sysfs_attrs);
	if (error) {
		dev_err(&pdev->dev, "Failed to create sysfs entries");
	}
	return error;
}

static void uio_sysfs_cleanup(struct platform_device *pdev) {
	sysfs_remove_files(&pdev->dev.kobj, uio_sysfs_attrs);
}

static irqreturn_t pruss_handler(int irq, struct uio_info *info)
{
	struct uio_pruss_dev *gdev = info->priv;
	int intr_bit = (irq - gdev->hostirq_start + 2);
	int val, intr_mask = (1 << intr_bit);
	void __iomem *base = gdev->prussio_vaddr + gdev->pintc_base;
	void __iomem *intren_reg = base + PINTC_HIER;
	void __iomem *intrdis_reg = base + PINTC_HIDISR;
	void __iomem *intrstat_reg = base + PINTC_HIPIR + (intr_bit << 2);

	val = ioread32(intren_reg);
	/* Is interrupt enabled and active ? */
	if (!(val & intr_mask) && (ioread32(intrstat_reg) & HIPIR_NOPEND))
		return IRQ_NONE;

#ifdef PRUSS_CHAR_DEVICE
	/* If the interruption corresponds to PRU_EVTOUT, we must signal
	 * other tasks, which might be waiting. */
	if (intr_bit == PRU_EVTOUT)
		complete(&intr_completion);
#endif

	/* Disable interrupt */
	iowrite32(intr_bit, intrdis_reg);
	return IRQ_HANDLED;
}

static void pruss_cleanup(struct platform_device *dev,
		struct uio_pruss_dev *gdev)
{
	int cnt;
	struct uio_info *p = gdev->info;

	uio_sysfs_cleanup(dev);

	for (cnt = 0; cnt < MAX_PRUSS_EVT; cnt++, p++) {
		uio_unregister_device(p);
		kfree(p->name);
	}
	iounmap(gdev->prussio_vaddr);
	if (gdev->ddr_vaddr) {
		dma_free_coherent(&dev->dev, extram_pool_sz, gdev->ddr_vaddr,
				gdev->ddr_paddr);
	}
#ifdef CONFIG_ARCH_DAVINCI_DA850
	if (gdev->sram_vaddr)
		gen_pool_free(gdev->sram_pool,
				gdev->sram_vaddr,
				sram_pool_sz);
#endif
	kfree(gdev->info);
	clk_put(gdev->pruss_clk);
	kfree(gdev);
}

static int pruss_probe(struct platform_device *dev)
{
	struct uio_info *p;
	struct uio_pruss_dev *gdev;
	struct resource *regs_prussio;
	struct resource res;
	int ret = -ENODEV, cnt = 0, len;
	struct uio_pruss_pdata *pdata = dev->dev.platform_data;
	struct pinctrl *pinctrl;

	int count;
	struct device_node *child;
	const char *pin_name;

#ifdef PRUSS_CHAR_DEVICE
	/* Saves platform_device which was detected by the system */
	_pdev = dev;
	_pdev_c++;
#endif

	gdev = kzalloc(sizeof(struct uio_pruss_dev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	gdev->info = kzalloc(sizeof(*p) * MAX_PRUSS_EVT, GFP_KERNEL);
	if (!gdev->info) {
		kfree(gdev);
		return -ENOMEM;
	}
#ifdef CONFIG_ARCH_DAVINCI_DA850
	/* Power on PRU in case its not done as part of boot-loader */
	gdev->pruss_clk = clk_get(&dev->dev, "pruss");
	if (IS_ERR(gdev->pruss_clk)) {
		dev_err(&dev->dev, "Failed to get clock\n");
		kfree(gdev->info);
		kfree(gdev);
		ret = PTR_ERR(gdev->pruss_clk);
		return ret;
	} else {
		clk_enable(gdev->pruss_clk);
	}
#endif

	if (dev->dev.of_node) {
		pm_runtime_enable(&dev->dev);
		ret = pm_runtime_get_sync(&dev->dev);
		if (IS_ERR_VALUE(ret)) {
			dev_err(&dev->dev, "pm_runtime_get_sync() failed\n");
			return ret;
		}

		ret = of_address_to_resource(dev->dev.of_node, 0, &res);
		if (IS_ERR_VALUE(ret)) {
			dev_err(&dev->dev, "failed to parse DT reg\n");
			return ret;
		}
		regs_prussio = &res;
	}

	pinctrl = devm_pinctrl_get_select_default(&dev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&dev->dev,
				"pins are not configured from the driver\n");

	// Run through all children. They have lables for easy reference.
	for_each_child_of_node(dev->dev.of_node, child) {
		enum of_gpio_flags flags;
		unsigned gpio;

		count = of_gpio_count(child);

		ret = of_property_count_strings(child, "pin-names");
		if (ret < 0) {
			dev_err(&dev->dev, "Failed to get pin-names\n");
			continue;
		}
		if(count != ret){
			dev_err(&dev->dev, "The number of gpios (%d) does not match"\
					" the number of pin names (%d)\n", count, ret);
			continue;
		}

		dev_dbg(&dev->dev, "Child has %u gpios\n", count);
		for(cnt=0; cnt<count; cnt++){
			ret = of_property_read_string_index(child,
					"pin-names", cnt, &pin_name);
			if (ret != 0)
				dev_err(&dev->dev, "Error on pin-name #%d\n", cnt);
			gpio = of_get_gpio_flags(child, cnt, &flags);
			ret = devm_gpio_request_one(&dev->dev, gpio, flags, pin_name);
		}
	}

	regs_prussio = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!regs_prussio) {
		dev_err(&dev->dev, "No PRUSS I/O resource specified\n");
		goto out_free;
	}

	if (!regs_prussio->start) {
		dev_err(&dev->dev, "Invalid memory resource\n");
		goto out_free;
	}

	if (pdata && pdata->sram_pool) {
		gdev->sram_pool = pdata->sram_pool;
#ifdef CONFIG_ARCH_DAVINCI_DA850
		gdev->sram_vaddr =
				gen_pool_alloc(gdev->sram_pool, sram_pool_sz);
		if (!gdev->sram_vaddr) {
			dev_err(&dev->dev, "Could not allocate SRAM pool\n");
			goto out_free;
		}
#endif
		gdev->sram_paddr =
				gen_pool_virt_to_phys(gdev->sram_pool,
						gdev->sram_vaddr);
	}

	gdev->ddr_vaddr = dma_alloc_coherent(&dev->dev, extram_pool_sz,
			&(gdev->ddr_paddr), GFP_KERNEL | GFP_DMA);
	if (!gdev->ddr_vaddr) {
		dev_err(&dev->dev, "Could not allocate external memory\n");
		goto out_free;
	}

	len = resource_size(regs_prussio);
	gdev->prussio_vaddr = ioremap(regs_prussio->start, len);
	if (!gdev->prussio_vaddr) {
		dev_err(&dev->dev, "Can't remap PRUSS I/O  address range\n");
		goto out_free;
	}

	if (dev->dev.of_node) {
		ret = of_property_read_u32(dev->dev.of_node,
				"ti,pintc-offset",
				&gdev->pintc_base);
		if (ret < 0) {
			dev_err(&dev->dev,
					"Can't parse ti,pintc-offset property\n");
			goto out_free;
		}
	} else
		gdev->pintc_base = pdata->pintc_base;

	gdev->hostirq_start = platform_get_irq(dev, 0);

	printk (KERN_INFO "gdev->hostirq_start %d", gdev->hostirq_start);

	for (cnt = 0, p = gdev->info; cnt < MAX_PRUSS_EVT; cnt++, p++) {
		p->mem[0].addr = regs_prussio->start;
		p->mem[0].size = resource_size(regs_prussio);
		p->mem[0].memtype = UIO_MEM_PHYS;

#ifdef CONFIG_ARCH_DAVINCI_DA850
		p->mem[1].addr = gdev->sram_paddr;
		p->mem[1].size = sram_pool_sz;
		p->mem[1].memtype = UIO_MEM_PHYS;

		p->mem[2].addr = gdev->ddr_paddr;
		p->mem[2].size = extram_pool_sz;
		p->mem[2].memtype = UIO_MEM_PHYS;
#else
		p->mem[1].addr = gdev->ddr_paddr;
		p->mem[1].size = extram_pool_sz;
		p->mem[1].memtype = UIO_MEM_PHYS;
#endif

		p->name = kasprintf(GFP_KERNEL, "pruss_evt%d", cnt);
		p->version = DRV_VERSION;

		/* Register PRUSS IRQ lines */
		p->irq = gdev->hostirq_start + cnt;
		p->handler = pruss_handler;
		p->priv = gdev;

		ret = uio_register_device(&dev->dev, p);
		if (ret < 0)
			goto out_free;
	}

	if (uio_sysfs_init(dev))
		goto out_free;

	platform_set_drvdata(dev, gdev);
	return 0;

	out_free:
	pruss_cleanup(dev, gdev);
	return ret;
}

static int pruss_remove(struct platform_device *dev)
{
	struct uio_pruss_dev *gdev = platform_get_drvdata(dev);

	pruss_cleanup(dev, gdev);
	platform_set_drvdata(dev, NULL);
	return 0;
}

static const struct of_device_id pruss_dt_ids[] = {
		{ .compatible = "ti,pruss-v1", .data = NULL, },
		{ .compatible = "ti,pruss-v2", .data = NULL, },
		{},
};
MODULE_DEVICE_TABLE(of, pruss_dt_ids);

static struct platform_driver pruss_driver = {
		.probe = pruss_probe,
		.remove = pruss_remove,
		.driver = {
				.name = DRV_NAME,
				.owner = THIS_MODULE,
				.of_match_table = pruss_dt_ids,
		},
};
/* Ending of standard uio_pruss driver */

/* Beginning of character device implementation and variables declaration */
#ifdef PRUSS_CHAR_DEVICE

#include <linux/mutex.h>
#include <asm/uaccess.h>

/* GPIO API used to recover hardware address */
#include <linux/gpio.h>

/* API support for variable completion */
#include <linux/completion.h>

#define  DEVICE_NAME "pruss485"
#define  CLASS_NAME  "pruss485"

/* Offset of memory areas and register offsets.
 * Refer to table 5 of the AM335x PRU Reference Guide*/
#define PRUSS_SHAREDRAM_BASE 0x10000
#define PINTC_HIEISR 0x0034
#define PRU_INTC_SECR1_REG 0x280
#define SRAM_SIZE 0x3000

/* ARM system interruption */
#define PRU_ARM_INTERRUPT 20

/* Application specific constants */
#define OLD_MESSAGE 0x55
#define	NEW_RECEIVED_MESSAGE 0x00
#define	MESSAGE_TO_SEND 0xff

/* GPIOs used to get board identification  */
#define GPIO_P8_31 10
#define GPIO_P8_32 11
#define GPIO_P8_33 9
#define GPIO_P8_34 81
#define GPIO_P8_35 8

/* ioctl() available commands */
enum ioctl_cmd {
	PRUSS_CLEAN = 10,
	PRUSS_MODE,
	PRUSS_SET_SYNC_STEP,
	PRUSS_SET_PULSE_COUNT_SYNC,
	PRUSS_GET_HW_ADDRESS,
	PRUSS_BAUDRATE,
	PRUSS_TIMEOUT,
	PRUSS_GET_PULSE_COUNT_SYNC,
	PRUSS_CLEAR_PULSE_COUNT_SYNC,
	PRUSS_START_SYNC,
	PRUSS_STOP_SYNC,
};

/* Shared RAM memory offsets */
enum offset {
	STATUS_OFFSET = 1,
	BAUD_BRGCONFIG_OFFSET,
	BAUD_LSB_OFFSET,
	BAUD_MSB_OFFSET,
	SYNC_OFFSET,
	TIMEOUT_OFFSET,
	HW_ADDR_OFFSET = 24,
	MODE_OFFSET,
	BAUD_LENGTH_OFFSET,
	INSTR_COUNT_OFFSET = 29,
	SYNC_STEP_OFFSET = 50,
	COUNTER_OFFSET = 80,
	SHRAM_WRITE_OFFSET = 0x64,
	SHRAM_READ_OFFSET = 0x1800,
};

static int majorNumber;
static u8 message[SRAM_SIZE] = {0};

/* mutex protecting read and writing order */
static DEFINE_MUTEX(pruchar_mutex);

static struct class* prucharClass  = NULL;
static struct device* prucharDevice = NULL;

/* struct file_operations function prototypes */
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static long    dev_unlocked_ioctl (struct file *, unsigned int, unsigned long);

/* application specific function prototypes */
static int init_gpio (unsigned int id, const char *);
static u8 dev_get_hw_addr (void);
static int dev_set_sync_counter (void __iomem *, unsigned long);
static int dev_clear_count_sync (void __iomem *);
static int dev_set_sync_stop (void __iomem *);
static int dev_set_sync_start (u32, void __iomem *);
static int dev_config_baudrate (void __iomem *, unsigned long);

/* file operations for file /dev/pru485 */
static struct file_operations fops = {
		.open = dev_open,
		.read = dev_read,
		.write = dev_write,
		.release = dev_release,
		.unlocked_ioctl = dev_unlocked_ioctl,
};

/* Character device functions */

/* Initialization procedure a GPIO pin */
static int init_gpio (unsigned int id, const char *label) {

	int err = 0;

	err |= gpio_request(id, label);
	err |= gpio_direction_input(id);

	return err;
}

/* Reads hardware address from GPIO pins */
static u8 dev_get_hw_addr(void) {

	u8 addr = 0;

	/* Initializes all GPIO ports */
	init_gpio(GPIO_P8_31, "gpio10");
	init_gpio(GPIO_P8_32, "gpio11");
	init_gpio(GPIO_P8_33, "gpio9");
	init_gpio(GPIO_P8_34, "gpio81");
	init_gpio(GPIO_P8_35, "gpio8");

	if (gpio_get_value(GPIO_P8_31))
		addr |= 0b0001;

	if (gpio_get_value(GPIO_P8_32))
		addr |= 0b0010;

	if (gpio_get_value(GPIO_P8_33))
		addr |= 0b0100;

	if (gpio_get_value(GPIO_P8_34))
		addr |= 0b1000;

	if (gpio_get_value(GPIO_P8_35))
		addr |= 0b10000;

	gpio_free(GPIO_P8_31);
	gpio_free(GPIO_P8_32);
	gpio_free(GPIO_P8_33);
	gpio_free(GPIO_P8_34);
	gpio_free(GPIO_P8_35);

	return addr;
}

/* Updates the synchronization counter with sync_counter parameter */
static int dev_set_sync_counter (void __iomem *io_vaddr, unsigned long sync_counter) {

	iowrite16(sync_counter, io_vaddr + COUNTER_OFFSET);

	return 0;
}

/* Bunch of commands needed to start sync operation */
static int dev_set_sync_start (u32 delay_us, void __iomem *io_vaddr) {

	if (ioread8(io_vaddr + MODE_OFFSET) == 'M') {

		u8 i;
		u32 delay_ns, n_loops;

		dev_clear_count_sync(io_vaddr);

		delay_ns = delay_us * 1000;

		/* Delay entre comando de sincronismo e requisicao qualquer */
		/* Calculo do delay */

		for (i = 0; i < 3; i++)
			delay_ns += (ioread8(io_vaddr + BAUD_LENGTH_OFFSET + i) << (i * 8));

		/* numero de loops = delay / 10 ns */
		n_loops = delay_ns/10;

		/* armazena numero de instrucoes */
		for (i = 0; i < 3; i++)
			iowrite8(n_loops >> (i * 8), io_vaddr + INSTR_COUNT_OFFSET + i);

		iowrite8(0xff, io_vaddr + SYNC_OFFSET);

		return 0;
	}

	return -1;
}

/* Configures serial baudrate */
static int dev_config_baudrate (void __iomem *io_vaddr, unsigned long baudrate) {

	u8 brgconfig, div_lsb, div_msb;
	int one_byte_length_ns;

	switch (baudrate) {

	case 6:
		brgconfig = 0x28;
		div_lsb = 0x02;
		div_msb = 0;
		one_byte_length_ns = 1667; /* 10000/6 */
		break;

	case 10:
		brgconfig = 0x28;
		div_lsb = 0x01;
		div_msb = 0;
		one_byte_length_ns = 1000; /* 10000/10 */
		break;

	case 12:
		brgconfig = 0x24;
		div_lsb = 0x01;
		div_msb = 0;
		one_byte_length_ns = 833; /* 10000/12 */
		break;

	case 9600:
		brgconfig = 0x0a;
		div_lsb = 0x86;
		div_msb = 0x01;
		one_byte_length_ns = 1041666; /* 100000000/96 */
		break;

	case 14400:
		brgconfig = 0x07;
		div_lsb = 0x04;
		div_msb = 0x01;
		one_byte_length_ns = 694444; /* 100000000/144 */
		break;

	case 19200:
		brgconfig = 0x05;
		div_lsb = 0xc3;
		div_msb = 0x00;
		one_byte_length_ns = 694444; /* 100000000/144 */
		break;

	case 38400:
		brgconfig = 0x15;
		div_lsb = 0xc3;
		div_msb = 0x00;
		one_byte_length_ns = 260416; /* 100000000/384 */
		break;

	case 57600:
		brgconfig = 0x27;
		div_lsb = 0x04;
		div_msb = 0x01;
		one_byte_length_ns = 173611; /* 100000000/576 */
		break;

	case 115200:
		brgconfig = 0x09;
		div_lsb = 0x20;
		div_msb = 0x00;
		one_byte_length_ns = 86805; /* 100000000/1152 */
		break;

	default:
		return -EINVAL;
	}

	iowrite8(brgconfig, io_vaddr + BAUD_BRGCONFIG_OFFSET);
	iowrite8(div_lsb, io_vaddr + BAUD_LSB_OFFSET);
	iowrite8(div_msb, io_vaddr + BAUD_MSB_OFFSET);

	iowrite8(one_byte_length_ns & 0xff, io_vaddr + BAUD_LENGTH_OFFSET);
	iowrite8((one_byte_length_ns >> 8) & 0xff, io_vaddr + BAUD_LENGTH_OFFSET + 1);
	iowrite8((one_byte_length_ns >> 16) & 0xff, io_vaddr + BAUD_LENGTH_OFFSET + 2);

	return 0;
}

/* Resets synchronization counter */
static int dev_clear_count_sync (void __iomem *io_vaddr) {

	if (!ioread8(io_vaddr + SYNC_OFFSET)) {

		iowrite16(0, io_vaddr + COUNTER_OFFSET);
		return 0;
	}

	/* Error: not possible to clear pulse counting while sync operation is enabled. */
	return -1;
}

/* Resets shared ram area */
static int dev_clean_sram (void __iomem *io_vaddr) {

	unsigned int count;

	for (count = 0; count < 100; count++)
		iowrite8(0, io_vaddr + count);

	return 0;
}

/* Stops sync operations. Node must be set as Master. */
static int dev_set_sync_stop (void __iomem *io_vaddr) {

	if (ioread8(io_vaddr + MODE_OFFSET) != 'M')
		return -1;

	iowrite8(0, io_vaddr + SYNC_OFFSET);

	return 0;

}

static int dev_set_sync_step (void __iomem *io_vaddr) {

	iowrite8(0x06, io_vaddr + SYNC_STEP_OFFSET);
	iowrite8(0xff, io_vaddr + SYNC_STEP_OFFSET + 1);
	iowrite8(0x50, io_vaddr + SYNC_STEP_OFFSET + 2);
	iowrite8(0x00, io_vaddr + SYNC_STEP_OFFSET + 3);
	iowrite8(0x01, io_vaddr + SYNC_STEP_OFFSET + 4);
	iowrite8(0x0c, io_vaddr + SYNC_STEP_OFFSET + 5);
	iowrite8(0xa4, io_vaddr + SYNC_STEP_OFFSET + 6);

	return 0;
}

/* Initialization procedure of the character device. Initializes mutexes and registers the device */
static int __init pru_driver_init(void) {

	printk(KERN_INFO "PRU KVM: initializing module.\n");

	/* Registering pruss_driver */
	platform_driver_register(&pruss_driver);
	_pdev_c = 0;

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

	printk(KERN_INFO "PRU KVM: device class created correctly\n");

	return 0;
}

/* Exits device and releases all resources. */
static void __exit pru_driver_exit(void) {

	platform_driver_unregister(&pruss_driver);

	mutex_destroy(&pruchar_mutex);

	device_destroy(prucharClass, MKDEV(majorNumber, 0));
	class_unregister(prucharClass);
	class_destroy(prucharClass);
	unregister_chrdev(majorNumber, DEVICE_NAME);
	printk(KERN_INFO "PRU KVM: module closed.\n");
}

/* Tries to acquire mutex and initializes the completion variable. Two or more
 * processes cannot open the file simultaneously */
static int dev_open(struct inode *inodep, struct file *filep){

	if(!mutex_trylock(&pruchar_mutex)){    /* Try to acquire the mutex */
		/* returns 1 if successful and 0 if there is contention */
		printk(KERN_ALERT "PRU KVM: Device in use by another process");
		return -EBUSY;
	}

	init_completion(&intr_completion);

	printk(KERN_INFO "PRU KVM: device has been opened.\n");
	return 0;
}

/* Releases resources after a close() call */
static int dev_release(struct inode *inodep, struct file *filep){

	mutex_unlock(&pruchar_mutex);

	printk(KERN_INFO "PRU KVM: device successfully closed.\n");
	return 0;
}

/* Reads current shared memory state  */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){

	int error_count = 0;

	if (_pdev) {

		struct uio_pruss_dev *gdev = platform_get_drvdata(_pdev);

		if (gdev) {

			struct resource *_regs_prussio = platform_get_resource(_pdev, IORESOURCE_MEM, 0);
			unsigned int _uio_size = resource_size(_regs_prussio), count = 0, i;
			void __iomem *p = ioremap(_regs_prussio->start, _uio_size) + PRUSS_SHAREDRAM_BASE;

			u8 mode = ioread8(p + MODE_OFFSET), status;

			printk(KERN_INFO "mode %c\n", mode);

			switch (mode) {
			case 'M':
				while (ioread8(p + STATUS_OFFSET)); //REPLACE?

				count = 0;

				printk(KERN_INFO "OKKKK\n");

				for (i = 0; i < 4; i++)
					count += (ioread8(p + SHRAM_READ_OFFSET) << (i*8));

				printk(KERN_INFO "65234523423\n");

				/*for (i = 0; i < count; i++)
					message[i] = ioread8(p + SHRAM_READ_OFFSET + 4 + i);
					*/

				for (count = 0;	count < SRAM_SIZE;	count++) {
					message[count] = ioread8(p + count);
					if (count < 100) printk (KERN_INFO "[%d] = 0x%u\n", count, message[count]);
				}

				break;

			case 'S':

				status = ioread8(p + STATUS_OFFSET);
				if (status  == NEW_RECEIVED_MESSAGE){

					count = 0;

					for (i = 0; i < 4; i++)
						count += (ioread8(p + SHRAM_READ_OFFSET) << (i*8));

					for (i = 0; i < count; i++)
						message[i] = ioread8(p + SHRAM_READ_OFFSET + 4 + i);

				}
				else if (status == OLD_MESSAGE)
					return -EINVAL;

				break;

			default:
				return -EINVAL;
			}

			/* copy_to_user has the format ( * to, *from, size) and returns 0 on success */
			if (copy_to_user(buffer, message, count)){
				printk(KERN_INFO "PRU KVM: Failed to send %d characters to the user\n", error_count);
				return -EFAULT;
			}

			printk(KERN_INFO "PRU KVM: Sent %d characters to the user\n", count);
			return 0;
		}
	}
	return -EINVAL;
}

/* Writes into the shared memory area */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){

	if (_pdev) {

		struct uio_pruss_dev *gdev = platform_get_drvdata(_pdev);

		if (gdev) {

			struct resource *_regs_prussio = platform_get_resource(_pdev, IORESOURCE_MEM, 0);
			unsigned int _uio_size = resource_size(_regs_prussio), count;
			void __iomem 	*base = ioremap(_regs_prussio->start, _uio_size),
					*p =  base + PRUSS_SHAREDRAM_BASE,
					*intrc = base + gdev->pintc_base;

			printk(KERN_INFO "EBBChar: Received %zu characters from the user\n", len);

			if (ioread8(p + MODE_OFFSET) == 'M') {
				//iowrite32(,p + TIMEOUT_OFFSET); TO-DO
			}

			/* SHRAM_WRITE_OFFSET is not 4-byte aligned, so we need to write
			 * each byte at a time */
			iowrite8(len & 0xff, p + SHRAM_WRITE_OFFSET);
			iowrite8((len >> 8) & 0xff, p + SHRAM_WRITE_OFFSET + 1);
			iowrite8((len >> 16) & 0xff, p + SHRAM_WRITE_OFFSET + 2);
			iowrite8((len >> 24) & 0xff, p + SHRAM_WRITE_OFFSET + 3);

			for (count = 0; count < len; count++)
				iowrite8(buffer[count], p + SHRAM_WRITE_OFFSET + 4 + count);

			iowrite8(MESSAGE_TO_SEND, p + STATUS_OFFSET);

			/* Waits for an interruption to finish the writing cycle. */
			wait_for_completion(&intr_completion);

			/* Clears system event */
			iowrite32(1 << PRU_ARM_INTERRUPT, intrc + PRU_INTC_SECR1_REG);

			/* Re-enables interruption */
			iowrite32(1 << PRU_EVTOUT, intrc + PINTC_HIEISR);

			if (ioread8(p + MODE_OFFSET) == 'M')
				while (ioread8(p + STATUS_OFFSET) != OLD_MESSAGE); /* REPLACE? */

			return len;

		}
		else {
			printk (KERN_INFO "gdev NULL \n");
			return -EFAULT;
		}
	}

	return -EINVAL;
}

/* ioctl() commands handler */
static long dev_unlocked_ioctl (struct file *filep, unsigned int cmd, unsigned long arg) {

	if (_pdev) {

		u8 hw_addr;
		struct uio_pruss_dev *gdev = platform_get_drvdata(_pdev);

		if (gdev) {

			struct resource *_regs_prussio = platform_get_resource(_pdev, IORESOURCE_MEM, 0);
			unsigned int _uio_size = resource_size(_regs_prussio);
			void __iomem *p = ioremap(_regs_prussio->start, _uio_size) + PRUSS_SHAREDRAM_BASE;

			switch (cmd) {

			case PRUSS_MODE:

				if (arg == 'M' || arg == 'S') {
					iowrite8(arg, p + MODE_OFFSET);

					dev_set_sync_stop(p);

					if (arg == 'S')
						iowrite8(OLD_MESSAGE, p + STATUS_OFFSET);

					return 0;
				}
				return -EINVAL;

			case PRUSS_BAUDRATE:

				return dev_config_baudrate(p, arg);

			case PRUSS_CLEAN:

				return dev_clean_sram(p);

			case PRUSS_GET_HW_ADDRESS:

				hw_addr = dev_get_hw_addr();
				iowrite8(hw_addr, p + HW_ADDR_OFFSET);

				return 0;

			case PRUSS_TIMEOUT:

				arg = arg * 66600;

				iowrite8(arg & 0xff, p + TIMEOUT_OFFSET);
				iowrite8((arg > 8) & 0xff, p + TIMEOUT_OFFSET + 1);
				iowrite8((arg > 16) & 0xff, p + TIMEOUT_OFFSET + 2);
				iowrite8((arg > 24) & 0xff, p + TIMEOUT_OFFSET + 3);

				return 0;

			case PRUSS_SET_SYNC_STEP:

				return dev_set_sync_step(p);

			case PRUSS_SET_PULSE_COUNT_SYNC:

				return dev_set_sync_counter(p, arg);

			case PRUSS_GET_PULSE_COUNT_SYNC:

				return ioread16(p + COUNTER_OFFSET);

			case PRUSS_CLEAR_PULSE_COUNT_SYNC:

				return dev_clear_count_sync(p);

			case PRUSS_START_SYNC:

				return dev_set_sync_start(arg, p);

			case PRUSS_STOP_SYNC:

				return dev_set_sync_stop(p);
			}
		}
		else return -EFAULT;
	}

	return -EINVAL;
}

module_init(pru_driver_init);
module_exit(pru_driver_exit);

MODULE_AUTHOR("Gustavo Ciotto Pinton <gustavo.pinton@lnls.br>");
#else
module_platform_driver(pruss_driver);
#endif

MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Amit Chatterjee <amit.chatterjee@ti.com>");
MODULE_AUTHOR("Pratheesh Gangadhar <pratheesh@ti.com>");
