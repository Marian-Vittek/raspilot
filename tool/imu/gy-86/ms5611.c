#include "common.h"
#include "MS5611.h"

MS5611 ms5611;

int main(int argc, char **argv) {
    int				i, streami;
    int64_t			sampleTime;
    struct raspilotTlibStr      ttt, *tt;

    tt = raspilotTlibInit(&ttt, argc, argv, TLIB_UNIVERSE_MAP_NO);
    streami = raspilotTlibInitStream(tt, (char*)"alt", TLIB_SHM_YES);
    
    if (tt->optSharedI2cFlag) pi2cInit(tt->optI2cPath, tt->optSharedI2cFlag);

    if (!ms5611.begin(tt->optI2cPath)) {
	fprintf(stderr, "%s:%f: Can't connect\n", __FILE__, __LINE__);
	return(-1);
    }

    for(;;) {
	// Read true temperature & Pressure
	double realTemperature = ms5611.readTemperature();
	long realPressure = ms5611.readPressure();
	sampleTime = raspilotTlibUsecTime();
 
	// Calculate altitude
	double absoluteAltitude = ms5611.getAltitude(realPressure);


	raspilotTlibSend(tt, streami, sampleTime, 1.0, &absoluteAltitude, 1);
	raspilotTlibMainLoopSleep(tt, sampleTime);
    }
 
}
