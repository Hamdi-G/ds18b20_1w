
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h> /*this is the file structure, file open read close */
#include <linux/cdev.h> /* this is for character device, makes cdev avilable*/
#include <linux/semaphore.h> /* this is for the semaphore*/
#include <linux/uaccess.h> /*this is for copy_user vice vers*/
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/platform.h>
#include <linux/io.h>
#include "ds18b20.h"

static int ds18b20_open(struct inode *, struct file *);
static int ds18b20_close(struct inode *, struct file *);
static ssize_t ds18b20_read(struct file *, char *, size_t, loff_t *);
static ssize_t ds18b20_write(struct file *, const char *, size_t, loff_t *);

static char device_buffer[BUFFER_SIZE] = "aartyaa i lovce you, i cant live without you";
int major_number; /* will store the major number extracted by dev_t*/
int ret; /*used to return values*/

struct semaphore sem;
struct cdev ds18b20_chr_dev; /*this is the name of my char driver that i will be registering*/
dev_t ds18b20_dev_num; /*will hold the major number that the kernel gives*/
static struct class *ds18b20_device_class;
struct device *device_info;


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

#if 1 
static int gpio_init_function(void)
{

	struct resource iomem;


	void __iomem *gpio_reg_base_addr = (void __iomem *)GPIO_REG_BASE;
	
	/*
	if( (base = ioremap_nocache(GPIO_REG_BASE, 32)) == NULL ) {
		printk( KERN_DEBUG "gpio_init_function : Failed to access the io region\n" );
		return -EACCES;

	}
	*/

	iomem.start = 0x20200000;
	iomem.end = 0x202000b3;
	
	printk("gpio_init_function : ioremapped address = 0x%08x\n", (unsigned int*)base);	
	printk("gpio_init_function : gpio_reg_base_addr = 0x%08x\n", (unsigned int*)gpio_reg_base_addr);	
	

	ds18b20_gpio_function(RPI_DQ_GPIO_PIN, FUN_OUTPUT);
	ds18b20_gpio_output(RPI_DQ_GPIO_PIN, OUTPUT_SET);	

	return 0;
	
}

static int reset_ds18b20_sensor(void)
{
	int ret = -1;

	printk(KERN_INFO "reset_ds18b20_sensor : setting gpio\n");
	gpio_init_function();
	
	return ret;
}

static int read_ds18b20_temp(void)
{
	int ret = -1;	
		
	reset_ds18b20_sensor();

	return ret;
}

#endif 

static ssize_t ds18b20_read(struct file *fp, char *buff, size_t length, loff_t *ppos) 
{
	int maxbytes;
	int bytes_to_read;
	int bytes_read;
	

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
    
	
    	read_ds18b20_temp();

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


failed_device_create :
	class_destroy(ds18b20_device_class);

failed_class_create:
	cdev_del(&ds18b20_chr_dev);

failed_device_add : 
	unregister_chrdev_region(ds18b20_dev_num, 1);

	return ret;
}

//void ds18b20_exit(void) 
static int remove_gpio(struct platform_device *pdev)
{
	device_destroy(ds18b20_device_class, ds18b20_dev_num);
	class_destroy(ds18b20_device_class);
	cdev_del(&ds18b20_chr_dev);
	unregister_chrdev_region(ds18b20_dev_num, 1);

	printk(KERN_DEBUG "ds18b20_exit : driver unregistered\n");

	return 0;
}


static int probe_ds18b20(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
        struct device_node *np = dev->of_node;
        struct local_gpio_data *local_data;
        struct resource iomem;
        int err, i;
        printk(KERN_INFO "probe_ds18b20 : calling the probe function\n");
        printk(KERN_INFO "%s : using the _func_ calling the probe function\n", __func__);

        local_data = devm_kzalloc(dev, sizeof(*local_data), GFP_KERNEL);
        if(!local_data)
                return -ENOMEM;

        printk(KERN_INFO "probe_ds18b20 : mem assigned \n");

        platform_set_drvdata(pdev, local_data);
        printk(KERN_INFO "probe_ds18b20 : set driver data and pdev->name = %s\n", pdev->name);

        local_data->dev = dev;

        err = of_address_to_resource(np, 0, &iomem);
        if (err) {

                dev_err(dev, "could not get IO memory\n");
                printk(KERN_INFO "probe_ds18b20 : could not get IO memory\n");
                return err;
        }
        printk(KERN_INFO "probe_ds18b20 : iomem.start = %x, iomem.end = %x\n", iomem.start, iomem.end);

        local_data->base = devm_ioremap_resource(dev, &iomem);

        printk(KERN_INFO "probe_ds18b20 : local_data->base = %x\n", local_data->base);
        if (IS_ERR(local_data->base))
                return PTR_ERR(local_data->base);

	return 0;
}

static struct platform_driver ds18b20_platform_data {

	.probe  = probe_ds18b20,
        .remove = remove_gpio,
        .driver =
        {
                .name   = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = of_device_id_gpio,            /*Struct used for matching a device */
        },

}

// module_init(ds18b20_init);
// module_exit(ds18b20_exit);


module_platform_data(ds18b20_platform_data);

MODULE_AUTHOR("AARTYAA");
MODULE_DESCRIPTION("A BASIC CHAR DRIVER");
MODULE_LICENSE("GPL");
