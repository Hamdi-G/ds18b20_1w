#ifndef _DS18B20_H_
#define _DS18B20_H_


#define BUFFER_SIZE     1024
#define DEVICENAME      "ds18b20"
#define DS18B20_DQ_GPIO_PIN     4       
#define HIGH    1
#define LOW     0
#define RPI_DQ_GPIO_PIN	4


#define GPIO_REG_BASE	0x7E200000
#define OUTPUT_SET_REG_1	0x7E20001C
#define OUTPUT_SET_REG_2	0x7E200020

#define GPIO_REG_PHY_BASE	0x20200000

#define FUN_INPUT	0x0  /* GPIO Pin  input */
#define FUN_OUTPUT	0x1  /* GPIO Pin output */
#define FUN_ALT_1	0x4  /* GPIO Pin alt 1 */
#define FUN_ALT_2	0x5  /* GPIO Pin alt 2 */
#define FUN_ALT_3	0x7  /* GPIO Pin alt 3 */

#define OUTPUT_SET	0x1
#define FUN_ALT0_SET	0x4
 
#define DS18B20_SEARCH	0xF0
#define DS18B20_SKIP_ROM	0xcc
#define DS18B20_CONVERT_TEMP	0x44
#define DS18B20_READ_SCRATCHPAD	0xBE

enum rpi_gpio4_state {low, high};
enum rpi_gpio4_direction {input, output};
void __iomem *base;

static u8 crc_table[] = {
        0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
        157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
        35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
        190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
        70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
        219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
        101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
        248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
        140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
        17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
        175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
        50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
        202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
        87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
        233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
        116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

struct ds18b20_data
{
	unsigned int pin;
        unsigned int ext_pullup_pin;
        unsigned int pullup_duration;
};

struct w1_reg_num
{
        __u64   family:8,
                id:48,
                crc:8;
};

#if 0
inline void write_to_reg(u32 b, volatile void __iomem *addr)
{
	printk(KERN_INFO "write_to_reg : address = %x, b = %ld\n", addr, b); 
        __raw_writel(b, addr);
        mb();
}

extern inline void ds18b20_gpio_function(u8 pin, u16 gpio_fun)
{       	
	u32 val; 
	
      	// void __iomem *base_addr = (void __iomem *)GPIO_REG_BASE;
      	void __iomem *base_addr = (void __iomem *)(0xf2200000);

	base_addr = 0xf2200000;
	printk("ds18b20_gpio_function : pin = %d, gpio_fun = %x\n", pin, gpio_fun);

	u8 offset = (pin / 10) * 4;	
	pin = (pin % 10) * 3; 	
	val =  gpio_fun << pin;	
	
	printk("ds18b20_gpio_function : pin = %d, val = %x, base_address + offset = %x\n", pin, val, base_addr + offset);

        write_to_reg(val, base_addr + offset);
}

extern inline void ds18b20_gpio_output(u8 pin, u8 gpio_output_set)
{
	u32 val; 
      	void __iomem *base_addr = (void __iomem *)OUTPUT_SET_REG_1;

	u8 offset = (pin / 32) * 4; 
	pin = (pin % 32);
	val = gpio_output_set << pin;	

	printk("ds18b20_gpio_output : offset = %x, pin = %d, val = %x, base_addr + offset = %x\n", offset, pin, base_addr + offset);
        
	write_to_reg(val, base_addr + offset);

}

#endif 
#endif
