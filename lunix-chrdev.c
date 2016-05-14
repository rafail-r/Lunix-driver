/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < Your name here >
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"
#define init_MUTEX(LOCKNAME) sema_init (LOCKNAME, 1);
/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;
/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state) {
	struct lunix_sensor_struct *sensor;
	printk("Entering the NEEDS_REFRESH\n");
	WARN_ON(!(sensor = state->sensor));
	printk("New time: %d Last receive time: %d Magic: %x\n",sensor->msr_data[state->type]->last_update,state->buf_timestamp,sensor->msr_data[state->type]->magic);
	if(state->buf_timestamp < sensor->msr_data[state->type]->last_update){	/*if there are recent new data (last_update > timestamb) return 1 */
		return 1;
	}
	return 0;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */

static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state) {
	struct lunix_sensor_struct *sensor;
	uint32_t time_stamp;
	uint32_t magic_no;
	uint32_t val;
	long lookup_value;
	int buf_pos = 0;	/*the position in the buffer*/
	int num;
	unsigned long flags;
	printk("Entering the UPDATE\n");
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	sensor = state->sensor;	/*our sensor from state struct*/
	spin_lock_irqsave(&sensor->lock,flags);
	/* Why use spinlocks? See LDD3, p. 119 */
	/*Otan h syskeyh stelnei dedomena kanei diakoph kai o epeksergasths ston handler paei sthn update sto lunix.protocols.c opoy pairnei ta dedomena kai
 	 *ta bazei ston sensor buffer mesw ths update sto lunix-sensors.c pou mesa exei spinlock (interrupt context). Emeis kanoume system call apo to user space 
	 *(process context) kai 8eloume na diabasoume sthn read -> update tous sensor buffers. Edw yparxei 8ema sygxronismou. Ta spinlocks einai busy-waits opou
	 * elegxei synexws an o allos to afhse kai ayto ginetai giati den einai diergasies na kanoun sleep den mporoume na kanoume sleep thn syskeyh*/
	/*
	 * Any new data available?
	 */
	magic_no = sensor->msr_data[state->type]->magic;
	time_stamp = sensor->msr_data[state->type]->last_update;
	val = sensor->msr_data[state->type]->values[0];
	spin_unlock_irqrestore(&sensor->lock,flags);			/*unlock*/
	printk("MAGIC: %x , TIMESTAMP:  %d , VAL: %d \n",magic_no,time_stamp,val);
	/* Now we can take our time to format them,
	 * holding only the private state semaphore*/
	if(magic_no != LUNIX_MSR_MAGIC || val >=65536){	/*If no magic data or strange val value error*/
		return -EFAULT;				/* Error code 14 Bad Address */
	}
	if( time_stamp <= state->buf_timestamp){ 	/* If these are not new data -> TRY AGAIN */
		return -EAGAIN;				/* Error code 11 Try Again */
	}
	if (state->type == BATT)
		lookup_value = lookup_voltage[val];
	else if (state->type == TEMP)
		lookup_value = lookup_temperature[val];
	else if (state->type == LIGHT)
		lookup_value = lookup_light[val];	/* take the value from the lookup table */
	else
		return -EFAULT;			/*if nothing from batt,temp and light there is an error*/
	if(lookup_value < 0) {			/*if value < 0 '-' char must be passed to buffer*/
		lookup_value *= -1;		/*to have a positive number*/
		state->buf_data[0] = '-';	/*'-' to buffer*/
		buf_pos = 1;			/*buffer position = 1*/
	}
	num = lookup_value/1000;		/* div 1000 to have the integer part*/
	if (num/10 != 0)
		state->buf_data[buf_pos++] = '0' + num/10;	/*copy the first digit to buffer*/
	state->buf_data[buf_pos++] = '0' + num%10;		/*copy the second digit to buffer*/
	state->buf_data[buf_pos++] = '.';			/*the '.'*/
	num = lookup_value%1000;				/* mod 1000 to have the rest number*/
	state->buf_data[buf_pos++] = '0' + num/100; 		/*copy the first digit (div 100)*/
        if (num%10 == 0){					/*if the last digit is 0 copy the second if it's not 0*/
		if ((num/10)%10 != 0)
			state->buf_data[buf_pos++] = '0' + (num/10)%10;	/*the second (div 10)mod(10)*/
	}
	else {							/*else all the digits*/
		state->buf_data[buf_pos++] = '0' + (num/10)%10; /*the second (div 10)mod(10)*/
		state->buf_data[buf_pos++] = '0' + num%10;	/*the last digit (mod10)*/
        }
	state->buf_data[buf_pos++] = '\n';			/*copy the 'next line' character to buffer*/
	state->buf_lim = buf_pos;				/*buffer limit = buffer position*/
	state->buf_timestamp = time_stamp;			/*copy the time_stamp to state struct*/
	debug("leaving UPDATE successfully\n");
	return 0;	/*return 0 if update successfull*/
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp) {
	/* Declarations */
	int ret;
	unsigned int minor_num;	/*the minor number of the divice*/
	struct lunix_chrdev_state_struct *private_state;
	debug("entering OPEN\n");
	ret = -ENODEV;	/*Error code 19 No such device*/
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;
	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	/* Allocate a new Lunix character device private state structure */
	minor_num = iminor(inode);	/*returns the minor num from inode the kernel's file descriptor*/
	private_state = kzalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);	/*allocate memory set to zero for private_state*/
	if(!private_state) {
		ret = -ENOMEM;	/*Error code 12: Out of memory*/
		printk(KERN_ERR "Memory allocation failed\n");	/*kernel print the error*/
		goto out;
	}
	printk("SENSOR: %d DEVICE: %d \n", minor_num%8 , minor_num>>3);
	/*allocate the device's state*/
	private_state->type = minor_num%8;
	private_state->sensor = &lunix_sensors[minor_num>>3];
	init_MUTEX(&(private_state->lock));
	filp->private_data = private_state;
out: 
	debug("leaving, with ret = %d\n", ret);
     	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp) {
	kfree(filp->private_data);	/*free when release file*/
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg) {
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret = 0;
	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;
	printk("Entering READ FUNCTION WITH filp: %p \n", filp);
	state = filp->private_data;	/*state struct from private_data*/
	WARN_ON(!state);
	sensor = state->sensor;	/*and our sensor from state struct*/
	WARN_ON(!sensor);
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if(down_interruptible(&state->lock)) { 
	/*This lock is in the same state-struct (the same fd for the open). For Examble a process and it's children
	 *state represends this file (lunix-temp etc.) for the process that opened it. If two processes open the
	 *lunix1-temp for example each process will have it's own fd, it's own state. This is not for sychronizing
	 *them but for sychronizing processes with the same state (fork). If states are different sychronizing is not
	 *needed each process in read will go to update and will take the current value.*/
        	return -ERESTARTSYS; 
        }
	if (*f_pos == 0) {
                printk("Needs a new value\n");
		while ((ret = lunix_chrdev_state_update(state)) == -EAGAIN) { /*if there is not new data wait*/
			up(&state->lock);	/*unlock because the processes are going to sleep*/
			if (filp->f_flags & O_NONBLOCK)
            			return -EAGAIN;	/*if nonblock flag then return "Try again error"*/
			/*all the waiting processes are sleeping until new data arrive. In lunix-sensors.c in update-sensors function there is
			 *a wake_up_interruptible command to wake up the processes after new data. The wait_event_interruptible checks if the new
			 *data arrived in the needs_refresh (if last_update>timestamb) and if 1 then ALL the waiting processes wake up. The first
			 *one will decrease the semaphore and lock for the other processes*/
			if(wait_event_interruptible(state->sensor->wq,lunix_chrdev_state_needs_refresh(state))){
				return -ERESTARTSYS;	/*if wake up from signal exit read returning error*/
			}
			if(down_interruptible(&state->lock)) {	/*The first process will lock*/
				return -ERESTARTSYS;
			}
			printk("Retrying for a new value\n");
		}
	}
	/* Determine the number of cached bytes to copy to userspace */
	if(ret) { 
		goto out; /*if ret!=0 return read returns ret value*/
	}

	if(*f_pos + cnt < state->buf_lim) {	/*If user call read function with small number of chars < buff_lim then read will not return the whole value*/
		cnt = state->buf_lim - cnt - *f_pos;
	} else {
		cnt = state->buf_lim - *f_pos;	/*cnt the rest characters*/
	}
	printk("%s %llu %zu \n",state->buf_data, *f_pos, cnt);
	if(copy_to_user(usrbuf, state->buf_data, cnt)){	/*copy the data to user*/
		printk("Copy_to_user failed \n");
		ret = -EFAULT;
		goto out;
	}
	ret = cnt;	/*read function returns the number of charcters*/
	*f_pos += cnt; 	/*f_pos will be increased so much as cnt*/
	if(*f_pos == state->buf_lim) {	/*if we have written the whole value f_pos will be 0 to take the next value in the next read*/
		*f_pos = 0;
		goto out;
	}
out:
	up(&state->lock);	/*unlock the semaphore for this state*/
	printk("Leaving READ\n");
	return ret;
	}

	static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma) {
		return -EINVAL;
	}

	static struct file_operations lunix_chrdev_fops = { .owner = THIS_MODULE,
		.open = lunix_chrdev_open,
		.release = lunix_chrdev_release,
		.read = lunix_chrdev_read,
		.unlocked_ioctl = lunix_chrdev_ioctl,
		.mmap = lunix_chrdev_mmap };

int lunix_chrdev_init(void) {
		/*
		 * Register the character device with the kernel, asking for
		 * a range of minor numbers (number of sensors * 8 measurements / sensor)
		 * beginning with LINUX_CHRDEV_MAJOR:0
		 */
		int ret;
		dev_t dev_no;
		unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

		debug("initializing character device\n");
		cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
		lunix_chrdev_cdev.owner = THIS_MODULE;

		dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
		/* register_chrdev_region? */
		ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix-tng");
		if (ret < 0) {
			debug("failed to register region, ret = %d\n", ret);
			goto out;
		}
		/* cdev_add? */
		ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
		if (ret < 0) {
			debug("failed to add character device\n");
			goto out_with_chrdev_region;
		}
		debug("completed successfully\n");
		return 0;

out_with_chrdev_region: unregister_chrdev_region(dev_no, lunix_minor_cnt);
out: return ret;
}

	void lunix_chrdev_destroy(void) {
		dev_t dev_no;
		unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

		debug("entering\n");
		dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
		cdev_del(&lunix_chrdev_cdev);
		unregister_chrdev_region(dev_no, lunix_minor_cnt);
		debug("leaving\n");
	}
