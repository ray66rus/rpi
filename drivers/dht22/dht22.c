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
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/atomic.h>

#include <linux/types.h>

#define DEV_NAME            "dht22"

#define WAIT_TIME 5 // wait time in microseconds for delays
#define SIGNAL_TOUT 22 // <timeout in microseconds> / WAIT_TIME

#define SENSOR_DATA_LENGTH 5
#define MAX_RESULT_STRING_LENGTH 40
#define RESULT_STRING_TEMPLATE "Temperature: %d.%d, Humidity: %d.%d\n"
#define CRC_ERROR_STRING "CRC error\n"

static int PIN = 4;
module_param(PIN, int, 0);

static short is_in_text_mode = 1;

static inline unsigned long long micros(void) {
	struct timeval t;
	do_gettimeofday(&t);
	return ((unsigned) t.tv_sec * (unsigned) 1000000) + t.tv_usec;
}

static inline void await(unsigned us) {
	unsigned long long await_timer = micros() + us;
	while(micros() < await_timer) ; 
}

static int dht22_open(struct inode *inode, struct file *file) {
    return nonseekable_open(inode, file);
}

static int dht22_release(struct inode *inode, struct file *file) {
    return 0;
}

static ssize_t dht22_write(struct file *file, const char __user *buf, size_t count, loff_t *pos) {
	if(count == 1)
		is_in_text_mode = 0;
	else
		is_in_text_mode = 1;
	return count;
}

static inline void _send_data_request(void) {
	gpio_direction_output(PIN, 0);
	await(1000);
	gpio_set_value(PIN, 1);
	gpio_direction_input(PIN);
}

static inline int _wait_for_state_change_with_tout(unsigned initial_state) {
	unsigned counter = SIGNAL_TOUT;
	while((gpio_get_value(PIN) == initial_state) && counter--)
		await(WAIT_TIME);
	return (counter > 0) ? 1 : 0;
	
}

static int _wait_for_high_with_tout(void) {
	return _wait_for_state_change_with_tout(0);
}

static int _wait_for_low_with_tout(void) {
	return _wait_for_state_change_with_tout(1);
}

static inline int _wait_for_data_start(void) {
	if(!_wait_for_high_with_tout())
		return 0;

	if(!_wait_for_low_with_tout())
		return 0;

	if(!_wait_for_high_with_tout())
		return 0;

	if(!_wait_for_low_with_tout())
		return 0;

	return 1;
}

static inline int _get_data(unsigned char *data) {
	int i,j;
	for(i=0;i<5;i++) {
		data[i] = 0;
		for(j=0;j<8;j++) {
			unsigned counter;

			if(!_wait_for_high_with_tout())
				return 0;

			counter = 0;
                        while(gpio_get_value(PIN) == 1 && counter < SIGNAL_TOUT) {
				counter++;
				await(WAIT_TIME);
                        }
			if(counter == SIGNAL_TOUT)
				return 0;
                        data[i] <<= 1;
                        if(counter > 7)
                                data[i] |= 0x01;
                }
        }

	return 1;
}

static inline int get_data_from_sensor(unsigned char *data) {
	_send_data_request();
	if(_wait_for_data_start() == 0)
		return -ETIMEDOUT;
	if(_get_data(data) == 0)
		return -ETIMEDOUT;
	return 0;
}

static size_t _get_result_as_text(unsigned char *data, char *res) {
	int t, h, t_int, h_int, t_dec, h_dec;

	if(((data[0] + data[1] + data[2] + data[3]) & 0xff) != data[4]) {
		strcpy(res, CRC_ERROR_STRING);
		return strlen(res);
	}

	t = ((data[2] & 0x7F) << 8) + data[3];
	h = (data[0] << 8) + data[1];
	t_int = t / 10;
	h_int = h / 10;
	t_dec = t - t_int*10;
	h_dec = h - h_int*10;
	if(data[2] & 0x80)
		t_int *= -1;

	sprintf(res, RESULT_STRING_TEMPLATE, t_int, t_dec, h_int, h_dec);
	return strlen(res);
}

static ssize_t dht22_read(struct file *file, char __user *buf, size_t count, loff_t *pos) {
	unsigned long flags;
	int err;
	size_t copy_result, data_length;
	unsigned char sensor_data[SENSOR_DATA_LENGTH];

	if(*pos != 0) {
		*pos = 0;
		return 0;
	}

	local_irq_save(flags);
	err = get_data_from_sensor(sensor_data);
	local_irq_restore(flags);

	if(err < 0)
		return err;

	if(is_in_text_mode) {
		char text_result[MAX_RESULT_STRING_LENGTH];
		data_length = _get_result_as_text(sensor_data, text_result);
		if(data_length > count)
			data_length = count;
		copy_result = copy_to_user(buf, text_result, data_length);
	} else {
		data_length = SENSOR_DATA_LENGTH;
		if(data_length > count)
			data_length = count;
		copy_result = copy_to_user(buf, sensor_data, data_length);
	}

	if(copy_result != 0)
		printk(KERN_ERR DEV_NAME ": %d bytes couldn't be copied to user space", copy_result);

	*pos = data_length - copy_result;
	return *pos;
}

static struct file_operations dht22_fops = {
	.owner = THIS_MODULE,
	.open = dht22_open,
	.read = dht22_read,
	.write = dht22_write,
	.release = dht22_release,
};

static struct miscdevice dht22_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEV_NAME,
	.fops = &dht22_fops,
};

static int __init dht22_init(void) {
	int request_result;

	printk(KERN_ERR DEV_NAME ": dht22 init with pin %d\n", PIN);

	request_result = gpio_request(PIN, "DHT22 pin");
	if(request_result) {
		printk(KERN_ERR DEV_NAME ": unable to request GPIO: %d\n", request_result);
		return request_result;
	}
	misc_register(&dht22_misc_device);
	return 0;
}

static void __exit dht22_exit(void) {
	printk(KERN_ERR DEV_NAME ": dht22_exit\n");
	misc_deregister(&dht22_misc_device);
	gpio_free(PIN);
}

subsys_initcall(dht22_init);
module_exit(dht22_exit);

MODULE_AUTHOR("Pavel Tolstov");
MODULE_DESCRIPTION("DHT22 driver");
MODULE_LICENSE("GPL");
