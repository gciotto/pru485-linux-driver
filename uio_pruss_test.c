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
	PRUSS_CLEAN = 10,
	PRUSS_MODE,
	PRUSS_SET_SYNC_STEP,
	PRUSS_SET_PULSE_COUNT_SYNC,
	PRUSS_GET_HW_ADDRESS,
	PRUSS_BAUDRATE,
	PRUSS_TIMEOUT,
	PRUSS_GET_PULSE_COUNT_SYNC,
	PRUSS_CLEAR_PULSE_COUNT_SYNC,
	PRUSS_START_SYNC,
	PRUSS_STOP_SYNC,
};

int main () {

	int ret, fd, i;
	char receive[SZ_12K];

	fd = open("/dev/pruss485", O_RDWR);

	ioctl(fd, PRUSS_CLEAN);
	ioctl(fd, PRUSS_MODE, 'M');
	ioctl(fd, PRUSS_SET_SYNC_STEP);
	ioctl(fd, PRUSS_SET_PULSE_COUNT_SYNC, 0);
	ioctl(fd, PRUSS_BAUDRATE, 12);
	ioctl(fd, PRUSS_GET_HW_ADDRESS);
	ioctl(fd, PRUSS_TIMEOUT, 8);
	ioctl(fd, PRUSS_SET_PULSE_COUNT_SYNC, 0x0102);
	printf("PRUSS_GET_PULSE_COUNT_SYNC = 0x%02x\n", ioctl(fd, PRUSS_GET_PULSE_COUNT_SYNC));

	printf("Reading from the device...\n");
	ret = read(fd, receive, SZ_12K);
	if (ret < 0){
		perror("Failed to read the message from the device.");
		return errno;
	}

	for (i = 0; i < 100; i++) {
		printf("[%2d] = 0x%02x ", i, receive[i]);

		if (!((i+1)%8))
			printf("\n");
	}

	printf("End of the program\n");

	close(fd);

	return 0;
}
