#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define PATH "/dev/ds18b20"

int main(int argc, char **argv)
{
	char buffer[1024];
	int devfile = -1;
	int read_sz = 0;

	devfile = open(PATH, O_RDONLY);

	if(devfile < 0)
	{
		perror("open");
		exit(-1);
	}

	read_sz = read(devfile, buffer, 1024);

	if(read_sz==0)
	{
		printf("No data available.\n");
		close(devfile);

		return 0;
	}
	puts(buffer);


	close(devfile);




return(0);

}

