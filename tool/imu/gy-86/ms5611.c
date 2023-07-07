#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <signal.h>

#include "pi2c.h"
#include "MS5611.h"

MS5611 ms5611;

int main(int argc, char **argv) {
    int		i, optSharedI2cFlag;
    char	*optI2cPath;
    double	optRate;

    optSharedI2cFlag = 0;
    optI2cPath = (char*)"/dev/i2c-1";
    optRate = 1000.0; 			// default rate 1kHz
    
    for(i=1; i<argc; i++) {
	if (strcmp(argv[i], "-s") == 0) {
	    // share i2c. Do not reset shared semaphores
	    optSharedI2cFlag = 1;
	} else if (strcmp(argv[i], "-r") == 0) {
	    // refresh rate in Hz
	    i++;
	    if (i<argc) optRate = strtod(argv[i], NULL);
	} else {
	    optI2cPath = argv[i];
	}
    }	
    
    if (optSharedI2cFlag) pi2cInit(optI2cPath, 1);


    if (!ms5611.begin(optI2cPath)) {
	fprintf(stderr, "%s:%f: Can't connect\n", __FILE__, __LINE__);
	return(-1);
    }

    for(;;) {
	// Read true temperature & Pressure
	double realTemperature = ms5611.readTemperature();
	long realPressure = ms5611.readPressure();
 
	// Calculate altitude
	float absoluteAltitude = ms5611.getAltitude(realPressure);

	printf("temp %g\n", realTemperature);
	printf("alt %g\n", absoluteAltitude);
	fflush(stdout);
    }
 
}
