#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <signal.h>

#include "pi2c.h"
#include "MPU6050.h"

static inline double doubleGetTime() {
  struct timespec tt;
  clock_gettime(CLOCK_REALTIME, &tt);
  return(tt.tv_sec + tt.tv_nsec/1000000000.0);
}

static void taskStop(int signum) {
    exit(0);
}

int main(int argc, char **argv) {
    double 	t0, t1, samplePeriod;
    int		i, usleepTime;
    int		magFd;
    uint8_t	mm[6];
    int16_t 	MgX,MgY,MgZ;

    int		optSharedI2cFlag;
    char	*optI2cPath;
    double	optRate;

    optSharedI2cFlag = 0;
    optI2cPath = (char*)"/dev/i2c-1";
    optRate = 50.0; 			// default rate 1kHz
    
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

    // create mpu connection
    MPU6050 	mpu(optI2cPath, 0x68);

    if (mpu.initialize() != 0) return(-1);

    // This allows hmc5883l wired on mpu6050
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true) ;
    mpu.setSleepEnabled(false);
    
    //mpu.MPU6050_write_reg (0x6A, 0);
    //mpu.MPU6050_write_reg (0x37, 2);
    //mpu.MPU6050_write_reg (0x6B, 0);
    
    signal(SIGINT, taskStop);
  
    usleep(1000);
    
    // connect to magnetometer
    magFd = pi2cOpen(optI2cPath, 0x1e);
    if (magFd < 0) {
	fprintf(stderr, "pi2c magnetometer connection failed\n");
	return(-1);
    }
    // 75Hz refresh rate
    pi2cWriteByteToReg(magFd, 0x00, 0x74);
    // gain
    pi2cWriteByteToReg(magFd, 0x01, 0x40);
    // continuous mode
    pi2cWriteByteToReg(magFd, 0x02, 0x00);
    
    usleepTime = 1000000 / optRate;
    usleep(100000);

    double mgPerDigit = 0.92f;

    t0 = doubleGetTime();
    i = 0;
    for(;;) {
	// read magnetometer
	// pi2cWriteBytesToReg(magFd, 0x03, 0, NULL);
	pi2cReadBytes(magFd, 0x03, 6, mm);
	t1 = doubleGetTime();

	MgX = ((int16_t)mm[0] << 8) | mm[1];
	MgY = ((int16_t)mm[2] << 8) | mm[3];
	MgZ = ((int16_t)mm[4] << 8) | mm[5];

	printf("mag %d %d %d\n", MgX, MgY, MgZ);
	fflush(stdout);

	samplePeriod = t1 - t0;
	t0 = t1;
	if (samplePeriod > 1.0/optRate && usleepTime > 0) usleepTime--;
	else if (samplePeriod < 1.0/optRate) usleepTime++;
	usleep(usleepTime);

	// if (i++ % 1000 == 0) printf("debug usleepTime == %d\n", usleepTime);
    }

    taskStop(0);
}
