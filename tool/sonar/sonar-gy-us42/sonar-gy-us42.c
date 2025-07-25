/*
  Measure the distance using gy-us42 i2c sensor and Raspberry Pi.
  This program uses pigpio library.
 */

#include "common.h"

int main(int argc, char **argv) {
    int				sonari, streami;
    uint8_t			b[2];
    uint16_t			distance;
    double			dd;
    int64_t			sampleTime;
    struct raspilotTlibStr      ttt, *tt;

    tt = raspilotTlibInit(&ttt, argc, argv, TLIB_UNIVERSE_MAP_NO);
    streami = raspilotTlibInitStream(tt, (char*)"dist", TLIB_SHM_YES);
    sonari = raspilotTlibInitI2cDevice(tt, 0x70);
    
    for(;;) {
	
	// get a messsurement
	pi2cReadBytesWithDelay(tt->i2c[sonari], 0x51, 60000, 2, b);
	sampleTime = raspilotTlibUsecTime();
	
	// my sensor continuously sets the first bit even if meassurements seem correct, so filter the bit
	b[0] = b[0] & 0x7f;
	distance = b[0] * 256 + b[1];
	// translate to m
	dd = distance / 100.0;

	// push to output
	raspilotTlibSend(tt, streami, sampleTime, 1.0, &dd, 1);

	// sleep depending on user defined rate
	raspilotTlibMainLoopSleep(tt, sampleTime);
    }
    

}
