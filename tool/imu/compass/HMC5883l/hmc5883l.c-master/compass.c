#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "hmc5883l.h"

static int file;

// find magnetic declination at
// https://www.magnetic-declination.com/
// Bratislava Slovakia: +5° 14'

#define MAGNETIC_DECLINATION ((5.0 + 14.0/60.0)/180.0 * M_PI)

static int closeAndExit(int code)
{
	close(file);
	return code;
}

int main(int argc, char **argv) {
    char   	*i2cfile;
    int		i, calibration, xmin, xmax, ymin, ymax;
    short 	x, y, z;
    double 	heading;

    calibration = 0;
    if (argc > 2) {
	printf("debug: %s:%d: Entering Calibration mode. Move compass for 10 seconds\n", __FILE__, __LINE__);
	calibration = 1;
	xmin = ymin = (1<<17);
	xmax = ymax = -(1<<17);
    } else {
	xmin = -520;
	xmax = 766;
	ymin = -720;
	ymax = 410;
	
	 xmin = -153;
	 xmax = 254;
	 ymin = -308;
	 ymax = 71;

	 // xmax = xmin = ymax = ymin = 0;
    }

    i2cfile = "/dev/i2c-1";
    if (argc > 1) i2cfile = argv[1];
    
    file = open(i2cfile, O_RDWR);
    if (file < 0)
	return -1;

    if (HMC5883L_Init(file, HMC5883L_ID, true))
	return closeAndExit(-1);

    struct HMC5883L conf = {
        .gain = HMC5883L_GAIN_1090,
        .measurementMode = HMC5883L_MEASUREMENTMODE_NORMAL,
        .outputRate = (calibration?HMC5883L_OUTPUTRATE_75:HMC5883L_OUTPUTRATE_15),
        .samples = HMC5883L_SAMPLES_2,
    };
    if (HMC5883L_Configure(file, &conf))
	return closeAndExit(-1);

    if (HMC5883L_SetContinuousMeasurement(file))
	return closeAndExit(-1);

    for(i=0;; i++) {
	if (HMC5883L_ReadData(file, &x, &y, &z)) return closeAndExit(-1);
	if (calibration) {
	    if (x < xmin) xmin = x;
	    if (x > xmax) xmax = x;
	    if (y < ymin) ymin = y;
	    if (y > ymax) ymax = y;
	    if (i > 10000) {
		printf(" xmin = %d;\n", xmin);
		printf(" xmax = %d;\n", xmax);
		printf(" ymin = %d;\n", ymin);
		printf(" ymax = %d;\n", ymax);
		break;
	    }
	    usleep(1000);
	} else {
	    // The compass must be horizontal! All the time. We omit Z axis from calculus.
	    heading = atan2((y - ((ymax + ymin) / 2.0)), (x - ((xmax + xmin) / 2.0)));
	    heading += MAGNETIC_DECLINATION;
	    if (heading < 0) heading += 2*M_PI;
	    if (heading > 2*M_PI) heading -= 2*M_PI;

	    // North shall be 0, east 90, south 180, west 270, aka
	    // north 0, east M_PI/2, Soutch M_PI, West 3*M_PI/2.
	    printf("head 0 0 %5.2f\n", heading * 180/M_PI);
	    fflush(stdout);
	    usleep(100000);
	}
    }
    
    return closeAndExit(0);
}
