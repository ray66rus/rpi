/* Copyright (C) 2014, Pavel Tolstov
 * Copyright (C) 2013, Jack Whitham
 * Copyright (C) 2009-2010, University of York
 * Copyright (C) 2004-2006, Advanced Micro Devices, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/compat.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/time.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/atomic.h>

#include <linux/types.h>

#define DEV_NAME            "testled"

#define PIN              4

#define BCM2708_PERI_BASE   0x20000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

static void __iomem *gpio;

static inline void INP_GPIO(unsigned g) {
	uintptr_t addr = (4 * (g / 10)) + (uintptr_t) gpio;
	iowrite32(ioread32((void *) addr) & ~(7<<(((g)%10)*3)), (void *) addr);
}

static inline void OUT_GPIO(unsigned g) {
	uintptr_t addr = (4 * (g / 10)) + (uintptr_t) gpio;
	iowrite32(ioread32((void *) addr) | (1<<(((g)%10)*3)), (void *) addr);
}

static inline void GPIO_SET(unsigned g) {
	iowrite32(g, (void *) (uintptr_t) gpio + (4 * 7));
}

static inline void GPIO_CLR(unsigned g) {
	iowrite32(g, (uintptr_t) gpio + (void *) (4 * 10));
}

static unsigned await_timer = 0;
static struct timeval initial;

static inline unsigned micros(void) {
	struct timeval t;
	do_gettimeofday(&t);
	t.tv_sec -= initial.tv_sec;
	return ((unsigned) t.tv_sec * (unsigned) 1000000) + t.tv_usec;
}

static inline void await(unsigned us) {
	await_timer = micros() + us;
	while(micros() < await_timer) ; 
}

static int testled_open(struct inode *inode, struct file *file) {
    return nonseekable_open(inode, file);
}

static int testled_release(struct inode *inode, struct file *file) {
    return 0;
}

static inline void blink(void) {
	GPIO_SET(1 << PIN);
	await(1000000);
	GPIO_CLR(1 << PIN);
}

static ssize_t testled_write(struct file *file, const char __user *buf, size_t count, loff_t *pos) {
	long val = 0;
	unsigned code = 0;
	unsigned timing = 0;
	char tmp[9];
	unsigned long flags;

	printk(KERN_ERR DEV_NAME ": trying to blink");

	local_irq_save(flags);
	blink();
	local_irq_restore(flags);
	
/*	if (count < 8)
	        return -EINVAL;

	if(copy_from_user(tmp, buf, 8))
	        return -EFAULT;

	tmp[8] = '\0';
	if (kstrtol(tmp, 16, &val) != 0)
		return -EINVAL;

	code = val;
	if ((code == 0) || (code >= (1 << 28)))
		return -EINVAL;
*/
//	local_irq_save(flags);
//	timing = transmit_code(code, 10);
//	local_irq_restore(flags);
//	printk(KERN_ERR DEV_NAME ": send %08x took %u\n", code, timing);
	return count;
}

static ssize_t testled_read(struct file *file, char __user *buf, size_t count, loff_t *pos) {
	return -EINVAL;
}

static struct file_operations testled_fops = {
	.owner = THIS_MODULE,
	.open = testled_open,
	.read = testled_read,
	.write = testled_write,
	.release = testled_release,
};

static struct miscdevice testled_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEV_NAME,
	.fops = &testled_fops,
};

static int __init testled_init(void) {
	unsigned long flags;

	printk(KERN_ERR DEV_NAME ": testled_init\n");

	gpio = ioremap_nocache(GPIO_BASE, BLOCK_SIZE);
	if(gpio == NULL) {
		printk(KERN_ERR DEV_NAME ": ioremap_nocache failed\n");
		return -ENXIO;
	}
	printk(KERN_ERR DEV_NAME ": your number is %p\n", gpio);
	misc_register(&testled_misc_device);
	local_irq_save(flags);
	INP_GPIO(PIN);
	OUT_GPIO(PIN);
	local_irq_restore(flags);
	return 0;
}

static void __exit testled_exit(void) {
	printk(KERN_ERR DEV_NAME ": testled_exit\n");
	iounmap(gpio);
	misc_deregister(&testled_misc_device);
	gpio = NULL;
}

subsys_initcall(testled_init);
module_exit(testled_exit);

MODULE_AUTHOR("Pavel Tolstov");
MODULE_DESCRIPTION("test led driver");
MODULE_LICENSE("GPL");
