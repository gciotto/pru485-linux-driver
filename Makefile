obj-m+=pru485_driver.o uio_pruss.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
	$(CC) -o test test.c
	$(CC) uio_pruss_test.c -o pru_test -lPRUserial485 -lprussdrv
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
	rm test
