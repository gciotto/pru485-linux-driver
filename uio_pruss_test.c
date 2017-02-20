#include <prussdrv.h>
#include <PRUserial485.h>

int main () {

	init_start_PRU(9600, 'M');

	close_PRU();

	return 0;
}
