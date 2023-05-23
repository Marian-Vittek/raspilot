#include "bmp180.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

void *bmp;

void signalInt(int signum) {
    bmp180_close(bmp);
    exit(0);
}

int main(int argc, char **argv){
    char 	*i2c_device = "/dev/i2c-11";
    double	frequency = 20;
    int 	address = 0x77;

    if (argc >= 2) i2c_device = argv[1];
    if (argc >= 3) frequency = atof(argv[2]);
    
    bmp = bmp180_init(address, i2c_device);
    if(bmp == NULL) return(-1);

    bmp180_eprom_t eprom;
    bmp180_dump_eprom(bmp, &eprom);
    bmp180_set_oss(bmp, 1);

    signal(SIGINT, signalInt);

    for(;;) {
	// long p = bmp180_pressure(bmp);
	float t = bmp180_temperature(bmp);
	float alt = bmp180_altitude(bmp);
	printf("alt %.2f\n", alt);
	printf("temp %5.2f\n", t);
	// Don't forget to fflush because auto flushing on stdout does not work when writing to a pipe.
	fflush(stdout);
	// repeat (approximately) at the given frequency
	usleep(1000000/frequency);
    }
    signalInt(0);	
    return 0;
}
