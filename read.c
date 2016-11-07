
//READING FILE

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define PATH "/dev/ds18b20"

int main(int argc, char **argv)
{
	char buffer[1024];
	int devfile = -1;
	int read_sz = 0,  temp = 0 ;

	devfile = open(PATH, O_RDONLY);

	if(devfile < 0)
	{
		perror("open");
		exit(-1);
	}

	read_sz = read(devfile, buffer, 1024);

	if(read_sz < 0)
	{
		printf("No data available.\n");
		close(devfile);

		return 0;
	}
	// printf("data read1  = %d, data read2 = %d\n", buffer[0], buffer[1], read_sz);
	
	temp = ( (buffer[0] << 8) | buffer[1]);
	
	printf("DS18B20 TEMP = %f\n", temp/1000.0);

	close(devfile);

return(0);

}

