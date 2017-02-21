#include <prussdrv.h>
#include <PRUserial485.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define SZ_12K 0x3000

enum ioctl_cmd {
	PRUSS_CLEAN,
	PRUSS_MODE,
	PRUSS_BAUDRATE,
	PRUSS_SYNC_STEP,
	PRUSS_SET_COUNTER,
};

int main () {

	int ret, fd, i;

	fd = open("/dev/pruss485", O_RDWR);

	ioctl(fd, PRUSS_CLEAN);
	ioctl(fd, PRUSS_MODE, 'M');
	ioctl(fd, PRUSS_SET_COUNTER, 0);

	ioctl(fd, PRUSS_BAUDRATE, 14400);
	ioctl(fd, PRUSS_SYNC_STEP);

	char receive[SZ_12K];

	printf("Reading from the device...\n");
	ret = read(fd, receive, SZ_12K);
	if (ret < 0){
		perror("Failed to read the message from the device.");
		return errno;
	}


	for (i = 0; i < SZ_12K; i++) {
		printf("[%d] = 0x%x ", i, receive[i]);

		if (!((i+1)%10))
			printf("\n");

	}

	printf("End of the program\n");

	close(fd);

	return 0;
}
