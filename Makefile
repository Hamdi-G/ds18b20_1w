#obj-m += pritam.o aartyaa.o
# obj-m += ds18b20.o 

CROSS_COMPILE = arm-linux-gnueabihf-
RPI_COMPILE_OPTION = ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)
GCC = gcc
SOURCE_READ_CODE = read.c
SOURCE_WRITE_CODE = write.c
READ_EXE = DS18B20_TEMP_READ
WRITE_EXE = DS18B20_TEMP_WRITE

#KERNEL_DIR = /home/prityaa/Documents/workspace/embeded/raspbery_pi/resources/kernel/linux-rpi-4.4.y

KERNEL_DIR = /home/prityaa/documents/workspace/embeded/raspbery_pi/code_base/gpio_static_module/linux

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	$(GCC) -o $(READ_EXE) $(SOURCE_READ_CODE) 
	$(GCC) -o $(WRITE_EXE) $(SOURCE_WRITE_CODE) 

rpi:
	
	$(MAKE) $(RPI_COMPILE_OPTION) -C $(KERNEL_DIR) M=$(PWD) modules
	$(CROSS_COMPILE)$(GCC) -o $(READ_EXE) $(SOURCE_READ_CODE) 
	$(CROSS_COMPILE)$(GCC) -o $(WRITE_EXE) $(SOURCE_WRITE_CODE) 


rpi_clean:
	$(MAKE) $(RPI_COMPILE_OPTION) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -rf $(READ_EXE) $(WRITE_EXE)


clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

	#$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -rf $(READ_EXE) $(WRITE_EXE)

