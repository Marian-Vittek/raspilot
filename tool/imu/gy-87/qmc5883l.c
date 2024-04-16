#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <signal.h>

#include "pi2c.h"
#include "MPU6050.h"

/* The default I2C address of this chip */
#define QMC5883L_ADDR 0x0D

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64  0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ   0b00000000
#define QMC5883L_CONFIG_50HZ   0b00000100
#define QMC5883L_CONFIG_100HZ  0b00001000
#define QMC5883L_CONFIG_200HZ  0b00001100

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT    0b00000001


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
    int		i, r, usleepTime;
    int		magFd;
    uint8_t	mm[6];
    int16_t 	MgX,MgY,MgZ;

    char	*calibrationFile;
    
    int		optSharedI2cFlag;
    char	*optI2cPath;
    double	optRate;

    
    optSharedI2cFlag = 0;
    optI2cPath = (char*)"/dev/i2c-1";
    optRate = 50.0;
    
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

    // This allows bypass to access h(q)mc5883l wired on mpu6050
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true) ;
    mpu.setSleepEnabled(false);
    
    //mpu.MPU6050_write_reg (0x6A, 0);
    //mpu.MPU6050_write_reg (0x37, 2);
    //mpu.MPU6050_write_reg (0x6B, 0);
    
    signal(SIGINT, taskStop);
  
    usleep(1000);
    
    // connect to magnetometer
    magFd = pi2cOpen(optI2cPath, 0x0d);
    if (magFd < 0) {
	fprintf(stderr, "pi2c magnetometer connection failed\n");
	return(-1);
    }
    
    pi2cWriteByteToReg(magFd, QMC5883L_RESET, 0x01);
    pi2cWriteByteToReg(magFd, QMC5883L_CONFIG,  QMC5883L_CONFIG_OS512 | QMC5883L_CONFIG_2GAUSS | QMC5883L_CONFIG_50HZ | QMC5883L_CONFIG_CONT);

    usleepTime = 1000000 / optRate;
    usleep(100000);

    t0 = doubleGetTime();
    i = 0;
    for(;;) {
	// read magnetometer
	uint8_t status;
	for(;;) {
	    pi2cReadBytes(magFd, QMC5883L_STATUS, 1, &status);
	    if (status & QMC5883L_STATUS_DRDY) break;
	    usleep(1000);
	}

	pi2cReadBytes(magFd, QMC5883L_X_LSB, 6, mm);
	t1 = doubleGetTime();
	MgX = ((int16_t)mm[1] << 8) | mm[0];
	MgY = ((int16_t)mm[3] << 8) | mm[2];
	MgZ = ((int16_t)mm[5] << 8) | mm[4];

	// return as roll, pitch, yaw. Raspilot shall be configured in the way that he knows that only yaw is valid.
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


