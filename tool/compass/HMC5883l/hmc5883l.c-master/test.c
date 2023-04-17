#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include "hmc5883l.h"

static int file;

static int closeAndExit(int code)
{
	close(file);
	return code;
}

int main(void) {
    int	i;
    short x, y, z;
    
    file = open("/dev/i2c-1", O_RDWR);
    if (file < 0)
	return -1;

    if (HMC5883L_Init(file, HMC5883L_ID, true))
	return closeAndExit(-1);

    struct HMC5883L conf = {
        .gain = HMC5883L_GAIN_1090,
        .measurementMode = HMC5883L_MEASUREMENTMODE_NORMAL,
        .outputRate = HMC5883L_OUTPUTRATE_15,
        .samples = HMC5883L_SAMPLES_2,
    };
    if (HMC5883L_Configure(file, &conf))
	return closeAndExit(-1);

    if (HMC5883L_SetContinuousMeasurement(file))
	return closeAndExit(-1);

    for(i=0; i<10000; i++) {
	if (HMC5883L_ReadData(file, &x, &y, &z)) return closeAndExit(-1);
	printf("%d %d %d\n", x,y,z);
	fflush(stdout);
	usleep(100000);
    }
    
    return closeAndExit(0);
}
