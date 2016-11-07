
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h> /*this is the file structure, file open read close */
#include <linux/cdev.h> /* this is for character device, makes cdev avilable*/
#include <linux/semaphore.h> /* this is for the semaphore*/
#include <linux/uaccess.h> /*this is for copy_user vice vers*/
#include <linux/cdev.h>
#include <linux/device.h>
// #include <mach/platform.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/delay.h>


#include "aartyaa_ds18b20.h"

static int ds18b20_open(struct inode *, struct file *);
static int ds18b20_close(struct inode *, struct file *);
static ssize_t ds18b20_read(struct file *, char *, size_t, loff_t *);
static ssize_t ds18b20_write(struct file *, const char *, size_t, loff_t *);

static int ds18b20_reset_bus(struct ds18b20_data *pdata);
static char device_buffer[BUFFER_SIZE] = "aartyaa i lovce you, i cant live without you";
int major_number; /* will store the major number extracted by dev_t*/
int ret; /*used to return values*/

struct semaphore sem;
struct cdev ds18b20_chr_dev; /*this is the name of my char driver that i will be registering*/
dev_t ds18b20_dev_num; /*will hold the major number that the kernel gives*/
static struct class *ds18b20_device_class;
struct device *device_info;
struct ds18b20_data data;

static int ds18b20_open(struct inode *inode, struct file *filp) 
{
	/*    
	if(down_interruptible(&sem) != 0) {
	        printk(KERN_ALERT "ds18b20_open : the device has been opened by some other device, unable to open lock\n");
        	return -1;
    	}
    	*/
	printk(KERN_INFO "ds18b20_open : device opened succesfully\n");
	return 0;
}

static int ds18b20_close(struct inode *inode, struct file *filp) 
{
	// up(&sem);
	printk(KERN_INFO "ds18b20_close : device has been closed\n");
	return 0;
}


static void ds18b20_gpio_set_dir(struct ds18b20_data *pdata, u8 bit)
{
        if (bit)
                gpio_direction_input(pdata->pin);
        else
                gpio_direction_output(pdata->pin, 0);
}


static u8 ds18b20_gpio_read_bit(struct ds18b20_data *pdata)
{
        return gpio_get_value(pdata->pin) ? 1 : 0;
}

static void ds18b20_write_bit(struct ds18b20_data *pdata, int bit)
{
        if (bit) {
		ds18b20_gpio_set_dir(pdata, 0);
		udelay(6);

		ds18b20_gpio_set_dir(pdata, 1);
		udelay(6);
        } else {
		ds18b20_gpio_set_dir(pdata, 0);
                udelay(60);

                ds18b20_gpio_set_dir(pdata, 1);
                udelay(10);
        }
}

static u8 ds18b20_read_bit(struct ds18b20_data *pdata)
{
        int result;

	ds18b20_gpio_set_dir(pdata, 0);
	udelay(6);

	ds18b20_gpio_set_dir(pdata, 1);
        udelay(9);

	result = ds18b20_gpio_read_bit(pdata);
       	udelay(55);

        return result & 0x1;
}


static u8 ds18b20_touch_bit(struct ds18b20_data *pdata, int bit)
{
        printk("ds18b20_touch_bit : bit = %d\n", bit);
	if (bit)
                return ds18b20_read_bit(pdata);
        else {
                ds18b20_write_bit(pdata, 0);
                return 0;
        }
}

u8 ds18b20_read_byte(struct ds18b20_data *pdata)
{
        int i;
        u8 res = 0;

	for (i = 0; i < 8; ++i)
		res |= (ds18b20_touch_bit(pdata, 1) << i);

	printk("ds18b20_read_byte : res = %x\n", res);

        return res;
}

void ds18b20_write_byte(struct ds18b20_data *pdata, u8 byte)
{
        int i;
        
	printk("ds18b20_write_byte : byte = %x\n", byte);
	for (i = 0; i < 8; ++i) 
		ds18b20_touch_bit(pdata, (byte >> i) & 0x1);
}

u8 ds18b20_read_block(struct ds18b20_data *pdata, u8 *buf, int len)
{
        int i;
        u8 ret;

        printk("ds18b20_read_block : pdata->pin = %d\n", pdata->pin);
        for (i = 0; i < len; ++i) {
                buf[i] = ds18b20_read_byte(pdata);
                printk("w1_read_block : buf[%d] = %x\n", i, buf[i]);
        }
        ret = len;


        return ret;
}

static inline int ds18b20_convert_temp(u8 rom[9])
{
        int t, h;

        if (!rom[7])
                return 0;

        if (rom[1] == 0)
                t = ((s32)rom[0] >> 1)*1000;
        else
                t = 1000*(-1*(s32)(0x100-rom[0]) >> 1);

        t -= 250;
        h = 1000*((s32)rom[7] - (s32)rom[6]);
        h /= (s32)rom[7];
        t += h;

        return t;
}

static inline int DS18B20_convert_temp(u8 rom[9])
{
        s16 t = le16_to_cpup((__le16 *)rom);
        return t*1000/16;
}

u8 w1_calc_crc8(u8 * data, int len)
{
        u8 crc = 0;

        while (len--)
                crc = crc_table[crc ^ *data++];

        return crc;
}

static int read_ds18b20_temp(char *temp_buf)  
{
	int ret = -1, i, count;
	u8 scratch_pad[9];
	unsigned int tm = 750;	
	unsigned long sleep_rem;
	int temp;
	
	printk("read_ds18b20_temp : pdata->pin = %d\n", data.pin);
	
	/* converting temp */
	if (ds18b20_reset_bus(&data ) )
		return -1; 
	ds18b20_write_byte(&data, DS18B20_SKIP_ROM);
	ds18b20_write_byte(&data, DS18B20_CONVERT_TEMP);
	
	/* reading scrtchpad */
	if (ds18b20_reset_bus(&data))
		return -2;
	ds18b20_write_byte(&data, DS18B20_SKIP_ROM);
	ds18b20_write_byte(&data, DS18B20_READ_SCRATCHPAD);
	if ((count = ds18b20_read_block(&data, scratch_pad, 9)) != 9) {
		printk("read_ds18b20_temp : failed to read scratch_pad\n");
		return -count;
	}

        printk("read_ds18b20_temp : printing rom : ");
	for (i = 0; i< 9; i++) 
  		printk("%02x ", scratch_pad[i]);
	
        printk("\n");
	
	if ( w1_calc_crc8(scratch_pad, 8 ) != scratch_pad[8] ) {
		printk("read_ds18b20_temp : failed to cal crc\n");
		return -3;
	}
	
	temp = DS18B20_convert_temp(scratch_pad);
	temp_buf[0] = temp >> 8;
	temp_buf[1] = temp & 0x00ff;

	printk("read_ds18b20_temp : temp = %d, temp_buf1 = %d, temp_buf2 = %d\n", temp, temp_buf[0], temp_buf[1]);

	return 0;
}


static int ds18b20_reset_bus(struct ds18b20_data *pdata)
{
	int result;

	printk("ds18b20_reset_bus : pin = %d\n", pdata->pin);	
	
	ds18b20_gpio_set_dir(pdata, 0);
	udelay(500);
	ds18b20_gpio_set_dir(pdata, 1);
	udelay(70);

	result = ds18b20_gpio_read_bit(pdata) & 0X1;
	msleep(1);
	
	return result;
}

u8 ds18b20_triplet(struct ds18b20_data *pdata, int bdir)
{

        u8 id_bit   = ds18b20_touch_bit(pdata, 1);
        u8 comp_bit = ds18b20_touch_bit(pdata, 1);
        u8 retval;

        printk("ds18b20_triplet : id_bit = %d, comp_bit = %d\n", id_bit, comp_bit);
        
	if (id_bit && comp_bit)
        	return 0x03;  /* error */

        if (!id_bit && !comp_bit) {
        /* Both bits are valid, take the direction given */
        	retval = bdir ? 0x04 : 0;
        } else {
	        /* Only one bit is valid, take that direction */
                bdir = id_bit;
                retval = id_bit ? 0x05 : 0x02;
        }

        printk("w1_triplet : bdir = %d, retval = %d\n", bdir, retval);

        ds18b20_write_bit(pdata, bdir);

        return retval;
}

static int ds18b20_ROM_search(struct ds18b20_data *p_data)
{
	int triplet_ret = -1, i;	
	u64 last_rn, rn = 0, tmp64;
	int last_zero;
	struct ds18b20_reg_num *tmp;
	
	if ( ds18b20_reset_bus(p_data) ) {
		printk("ds18b20_ROM_search : no device present on the bus\n");
		return -1;
	}
	
	/* Start the search */ 
        ds18b20_write_byte(p_data, DS18B20_SEARCH);	
	for (i = 0; i < 64; ++i) {
		/* Read two bits and write one bit */
		triplet_ret = ds18b20_triplet(p_data, DS18B20_SEARCH);
		printk("ds18b20_ROM_search : triplet_ret %d = %d\n", i, triplet_ret);

		/* quit if no device responded */
		if ( (triplet_ret & 0x03) == 0x03 )
			break;

		/* If both directions were valid, and we took the 0 path... */
		if (triplet_ret == 0)
			last_zero = i;

		/* extract the direction taken & update the device number */
		tmp64 = (triplet_ret >> 2);
		printk("ds18b20_ROM_search : tmp64 = %llx, triplet_ret = %d\n", tmp64, triplet_ret);

		rn |= (tmp64 << i);
		printk("rn = %llx\n", rn);
	}
	
	tmp = (struct ds18b20_reg_num *) &rn;

	printk("temp->crc = %x, temp->id = %llx, temp->family = %x\n", tmp->crc, tmp->id, tmp->family);
	return 0;
}

static int ds18b20_gpio_init(struct ds18b20_data *p_data)
{
	int ret = -1;

	/*
	if ( (data = kmalloc(sizeof (struct ds18b20_data), GFP_KERNEL) ) == NULL ) {
		printk("read_ds18b20_id : failed to alloc mem\n");
	}
	*/
	printk("ds18b20_gpio_init : pin = %d, p_data->ext_pullup_pin = %d\n", p_data->pin, p_data->ext_pullup_pin);
	
	ret = gpio_request(p_data->pin, "ds18b20");
        if (ret) {
		printk("ds18b20_gpio_init : failed to request gpio_pin = %d\n", p_data->pin);
                return -ret;
        }	
	
	printk("ds18b20_gpio_init : aartyaa\n");	
	ret = gpio_request_one(p_data->ext_pullup_pin, 0, "ds18b20 pullup");
        if (ret) {
                printk("ds18b20_gpio_init : failed to request gpio_pin = %d\n", p_data->ext_pullup_pin);
		return -ret;
        }

	gpio_direction_input(p_data->pin);
	
	return 0;
}

struct ds18b20_data data = {
	.pin = 4,
       	.ext_pullup_pin = 5,
       	.pullup_duration = 0,
};

static int read_ds18b20_id(void)
{

	printk("read_ds18b20_id : reading the id\n");

	if ( ds18b20_gpio_init(&data) < 0) {
		printk("read_ds18b20_id : failed to init gpio\n");
		return -1;
	}

	if ( ds18b20_ROM_search(&data) < 0) {
		printk("read_ds18b20_id : failed to search ROM\n");
		return -2;
	}

	return 0;
}

static ssize_t ds18b20_read(struct file *fp, char *buff, size_t length, loff_t *ppos) 
{
	int maxbytes, ret;
	int bytes_to_read;
	int bytes_read;
	char temp_buf[2];
	
	memset(temp_buf, '\0', sizeof(temp_buf));
	
    	if ( ! read_ds18b20_temp(temp_buf)) {
		ret = copy_to_user(buff, temp_buf, 2);	
		printk("ds18b20_read : ret = %d, temp_buf1 = %d, temp_buf2 %d\n", ret, temp_buf[0], temp_buf[1]);
		return ret;
	}
	
	maxbytes = BUFFER_SIZE - *ppos;
	if(maxbytes > length) 
        	bytes_to_read = length;
	else
        	bytes_to_read = maxbytes;
    
	if(bytes_to_read == 0)
        	printk(KERN_INFO "ds18b20_read : Reached the end of the device\n");
    
	bytes_read = bytes_to_read - copy_to_user(buff, device_buffer + *ppos, bytes_to_read);
    
	*ppos += bytes_read;
	printk(KERN_INFO "ds18b20_read : device has been read and buffer %s and flie position *ppos = %lld\n", buff, *ppos);
	printk(KERN_INFO "ds18b20_read : device has been read %d\n", bytes_read);
    
	return bytes_read;
}

static ssize_t ds18b20_write(struct file *fp, const char *buff, size_t length, loff_t *ppos) 
{
	int maxbytes; /*maximum bytes that can be read from ppos to BUFFER_SIZE*/
	int bytes_to_write; /* gives the number of bytes to write*/
	int bytes_writen;/*number of bytes actually writen*/

	maxbytes = BUFFER_SIZE - *ppos;

	if(maxbytes > length)
        	bytes_to_write = length;
	else
        	bytes_to_write = maxbytes;

	bytes_writen = bytes_to_write - copy_from_user(device_buffer + *ppos, buff, bytes_to_write);
    
	*ppos += bytes_writen;
	printk(KERN_INFO "ds18b20_write : device has been written and device_buffer %s, *ppos = %lld\n", buff, *ppos);
	printk(KERN_INFO "ds18b20_write : device has been written %d\n",bytes_writen);
	return bytes_writen;
}

struct file_operations fops = /* these are the file operations provided by our driver */
{ 							
	.owner = THIS_MODULE, 
	.open = ds18b20_open,
	.write = ds18b20_write,
	.read = ds18b20_read,
	.release = ds18b20_close,
};


int ds18b20_init(void) 
{
	int ret;

	if(!alloc_chrdev_region(&ds18b20_dev_num, 0, 1, DEVICENAME) ) {
		major_number = MAJOR(ds18b20_dev_num);
		
		cdev_init(&ds18b20_chr_dev, &fops);
        	ds18b20_chr_dev.owner = THIS_MODULE;

		if( !(ret = cdev_add(&ds18b20_chr_dev, ds18b20_dev_num, 1)) ) {
			if((ds18b20_device_class = class_create(THIS_MODULE, DEVICENAME)) != NULL ) {
				if ( (device_info = device_create(ds18b20_device_class, NULL, ds18b20_dev_num, NULL, "ds18b20")) != NULL ) {
					if (read_ds18b20_id() < 0 ) {
						printk("read_ds18b20_id : failed to search ROM\n");
				                ret = -4;	
						goto failed_read_ds18b20_id;
					}

					printk("ds18b20_init : device %s Created \n", DEVICENAME);
					printk(KERN_INFO "ds18b20_init : major number of our device is %d\n",major_number);
					return 0;
					// sema_init(&sem, 1);  /* initial value to one*/
		
				} else {
					printk("ds18b20_init : failed to create class\n");
					ret = -3; 
					goto failed_device_create;

				}

			} else {
				printk("ds18b20_init : failed to create class\n");
				ret = -2;
				goto failed_class_create;
			}
			
		}
		else if (ret  != 0) {
			printk("ds18b20_init : failed all the char dev to sys\n");
			ret = -1;
			goto failed_device_add;
		}	
		
	} else {
		printk("ds18b20_init : failed to allocate device \n");
		return -ENOMEM;
	}

failed_read_ds18b20_id:
	device_destroy(ds18b20_device_class, ds18b20_dev_num);

failed_device_create :
	class_destroy(ds18b20_device_class);

failed_class_create:
	cdev_del(&ds18b20_chr_dev);

failed_device_add : 
	unregister_chrdev_region(ds18b20_dev_num, 1);

	return ret;
}

void ds18b20_exit(void)
{
	device_destroy(ds18b20_device_class, ds18b20_dev_num);
	class_destroy(ds18b20_device_class);
	cdev_del(&ds18b20_chr_dev);
	unregister_chrdev_region(ds18b20_dev_num, 1);
	gpio_free(4);
	printk(KERN_DEBUG "ds18b20_exit : driver unregistered\n");
}

module_init(ds18b20_init);
module_exit(ds18b20_exit);

MODULE_AUTHOR("AARTYAA");
MODULE_DESCRIPTION("A BASIC CHAR DRIVER");
MODULE_LICENSE("GPL");
