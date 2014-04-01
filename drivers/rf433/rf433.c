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

#include "rfproto.h"

#define DEV_NAME            "rf433"

#define PIN_RX_DESC "RF433 RX pin"
#define PIN_TX_DESC "RF433 TX pin"
#define BUFFER_MAX_MESSAGES 2
#define BUFFER_SIZE BUFFER_MAX_MESSAGES * PACKET_DATA_LENGTH_WITH_SIZE 

static int PIN_TX = 17;
static int PIN_RX = 4;
module_param(PIN_TX, int, 0);
module_param(PIN_RX, int, 0);

static unsigned char buffer[BUFFER_SIZE];
static unsigned int first_msg_idx, current_msg_idx;
static DEFINE_SPINLOCK(buffer_lock);

static inline unsigned long long micros(void) {
	struct timeval t;
	do_gettimeofday(&t);
	return ((unsigned) t.tv_sec * (unsigned) 1000000) + t.tv_usec;
}

static inline void await(unsigned us) {
	unsigned long long await_timer = micros() + us;
	while(micros() < await_timer) ; 
}

short int irq_any_gpio = 0;

unsigned int _calc_crc8(unsigned char *buf, unsigned int len) {
        unsigned char crc = 0xFF;
        unsigned int i;

        while(len--) {
                crc ^= *buf++;
                for(i=0; i<8; i++) {
                        crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
                }
        }
        return crc;
}

unsigned int _is_crc8_ok(unsigned char *buf, unsigned int len, unsigned int crc8) {
        unsigned int new_crc8 = _calc_crc8(buf, len);
        return (crc8 == new_crc8) ? 1 : 0;
}

static inline unsigned _rf_get_next_bit(void) {
        int counter;
	counter = 0;
        for(counter=0;gpio_get_value(PIN_RX) == 1 && counter < CHANGE_STATE_TOUT;counter++)
                await(ATOMIC_DELAY);
        for(counter=0;gpio_get_value(PIN_RX) == 0 && counter < CHANGE_STATE_TOUT;counter++)
                await(ATOMIC_DELAY);
        return (counter > ATOMIC_DELAYS_IN_TIME_LOW) ? 1 : 0;
}

uint8_t _get_preamble_and_sync(void) {
        int current_bit = _rf_get_next_bit();
        int preamble_length = 1;
        int next_bit = current_bit;
	int i;
        do {
                current_bit = next_bit;
                next_bit = _rf_get_next_bit();
                preamble_length++;
        } while(current_bit != next_bit);
        if(preamble_length < 20 || current_bit == 0)
                return 0;
        for(i=0;i<6;i++)
                current_bit = _rf_get_next_bit();
        return (current_bit == 1) ? 1 : 0;
}

static inline int _receive_packet(unsigned char *buf) {
	int i,j;
	unsigned char data[PACKET_DATA_LENGTH_WITH_SIZE_AND_CRC];

	gpio_direction_input(PIN_RX);

	if(_get_preamble_and_sync() == 0)
		return -RF_ERR_TOUT;

        for(i=0;i<PACKET_DATA_LENGTH_WITH_SIZE_AND_CRC;i++) {
                data[i] = 0;
                for(j=0;j<8;j++) {
                        data[i] <<= 1;
                        data[i] |= _rf_get_next_bit();
                }
        }

	if(!_is_crc8_ok(data, PACKET_DATA_LENGTH + 1, data[PACKET_DATA_LENGTH + 1]))
		return -RF_ERR_CRC_ERROR;

	for(i=0;i<data[0]+1;i++)
		buf[i] = data[i];

        return data[0];
}

static void _add_received_packet_to_buffer(unsigned char *data) {
	unsigned char data_length_with_size;
	unsigned int in_msg_idx;
	int i;

	if(buffer[current_msg_idx] != 0) {
		current_msg_idx += buffer[current_msg_idx] + 1;
		if(current_msg_idx >= BUFFER_SIZE)
			current_msg_idx -= BUFFER_SIZE;
	}

	data_length_with_size = data[0] + 1;
	for(i=0, in_msg_idx=current_msg_idx;i<=data_length_with_size;i++) {
		buffer[in_msg_idx++] = data[i];
		if(in_msg_idx == BUFFER_SIZE)
			in_msg_idx = 0;
		if(i < data_length_with_size && in_msg_idx == first_msg_idx) {
			first_msg_idx += buffer[first_msg_idx] + 1;
			if(first_msg_idx >= BUFFER_SIZE)
				first_msg_idx -= BUFFER_SIZE;
		}
	}
}

static irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs) {
	unsigned long flags;
	unsigned char data[PACKET_DATA_LENGTH_WITH_SIZE];
	int data_length;

	local_irq_save(flags);
	data_length = _receive_packet(data);
	local_irq_restore(flags);

	if(data_length == -RF_ERR_TOUT)
		return IRQ_HANDLED;

	if(data_length == -RF_ERR_CRC_ERROR) {
		printk(KERN_NOTICE DEV_NAME ": CRC error\n");
		return IRQ_HANDLED;
	}

	if(data_length > PACKET_DATA_LENGTH_WITH_SIZE) {
		printk(KERN_NOTICE DEV_NAME ": wrong data length in incoming packet (%d), packet dropped\n", data_length);
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&buffer_lock, flags);
	_add_received_packet_to_buffer(data);
	spin_unlock_irqrestore(&buffer_lock, flags);

 
	return IRQ_HANDLED;
}
 
static int prepare_rx_pin(void) {
	int err;

	if((err = gpio_request(PIN_RX, PIN_RX_DESC)) < 0) {
		printk(KERN_ERR DEV_NAME ": unable to request RX GPIO pin\n");
		return err;
	}

	if((irq_any_gpio = gpio_to_irq(PIN_RX)) < 0 ) {
		printk(KERN_ERR DEV_NAME ": GPIO to irq mapping faiure\n");
		return irq_any_gpio;
	}

	printk(KERN_DEBUG DEV_NAME ": mapped irq %d\n", irq_any_gpio);

	if((err = request_irq(irq_any_gpio, (irq_handler_t ) r_irq_handler, IRQF_TRIGGER_FALLING, PIN_RX_DESC, NULL)) < 0) {
		printk(KERN_ERR DEV_NAME ": irq request failure\n");
		return err;
	}
 
	return 0;
}

static int prepare_tx_pin(void) {
	int err;

	err = gpio_request(PIN_TX, PIN_TX_DESC);
	if(err)
		printk(KERN_ERR DEV_NAME ": unable to request TX GPIO pin\n");
	return err;
}
 
void rx_pin_release(void) {
	gpio_free(PIN_RX);
	free_irq(irq_any_gpio, NULL);
}

void tx_pin_release(void) {
	gpio_free(PIN_TX);
}
 
static int rf433_open(struct inode *inode, struct file *file) {
	return nonseekable_open(inode, file);
}

static int rf433_release(struct inode *inode, struct file *file) {
	return 0;
}

static ssize_t rf433_write(struct file *file, const char __user *buf, size_t count, loff_t *pos) {
	return 0;
}

/*static inline void _send_data_request(void) {
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
}*/

static ssize_t rf433_read(struct file *file, char __user *buf, size_t count, loff_t *pos) {
	int i;
	char data[PACKET_DATA_LENGTH_WITH_SIZE];
	for(i=1;i<buffer[first_msg_idx];i++) {
		data[i] = buffer[first_msg_idx+i];
	}
	data[i] = 0;
	printk(KERN_ERR DEV_NAME ": buffer is: %u - %u %s\n", first_msg_idx, current_msg_idx, data);
/*	unsigned long flags;
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
	return *pos;*/
	return 0;
}

static struct file_operations rf433_fops = {
	.owner = THIS_MODULE,
	.open = rf433_open,
	.read = rf433_read,
	.write = rf433_write,
	.release = rf433_release,
};

static struct miscdevice rf433_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEV_NAME,
	.fops = &rf433_fops,
};

static int __init rf433_init(void) {
	int err;

	printk(KERN_NOTICE DEV_NAME ": RF433 init with pins %d (TX), %d (RX)\n", PIN_TX, PIN_RX);

	if((err = prepare_tx_pin()) < 0)
		return err;
	if((err = prepare_rx_pin()) < 0) {
		tx_pin_release();
		return err;
	}

	first_msg_idx = current_msg_idx = 0;
	buffer[current_msg_idx] = 0;

	misc_register(&rf433_misc_device);
	return 0;
}

static void __exit rf433_exit(void) {
	printk(KERN_NOTICE DEV_NAME ": RF433 exit\n");
	rx_pin_release();
	tx_pin_release();
	misc_deregister(&rf433_misc_device);
}

subsys_initcall(rf433_init);
module_exit(rf433_exit);

MODULE_AUTHOR("Pavel Tolstov");
MODULE_DESCRIPTION("RF433 RX/TX driver");
MODULE_LICENSE("GPL");
