
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#define PATH "/dev/ds18b20"

int main(int argc, char **argv)
{
	char buffer[100], *apliction_buffer;
	int devfile = -1;
	int req_sz = 100, len=0, ret = -1;
	
	printf("Enter the data \n");
	gets(buffer);
	
	len = strlen(buffer);

	apliction_buffer = (char *)malloc(len+1);	
	strcpy(apliction_buffer, buffer);

	devfile = open(PATH, O_WRONLY);
	if(devfile < 0) {
		perror("OPEN");
		exit(-1);
	}

	if((ret = write(devfile, apliction_buffer, strlen(apliction_buffer))) < 0)
		printf( "Device file has failed to write !\n" );
	else 
		printf( "Device file has been Written and ret %d \n", ret );
	
	close(devfile);

return(0);
}

