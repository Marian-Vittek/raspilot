#include "common.h"
#include "MPU6050.h"

static void taskStop(int signum) {
    exit(0);
}

int main(int argc, char **argv) {
    double 			t0, t1, samplePeriod;
    int64_t			sampleTime;
    int				i, streami;
    int				magFd;
    uint8_t			mm[6];
    int16_t 			MgX,MgY,MgZ;
    double			mxyz[3];
    struct raspilotTlibStr      ttt, *tt;


    tt = raspilotTlibInit(&ttt, argc, argv, TLIB_UNIVERSE_MAP_NO);
    streami = raspilotTlibInitStream(tt, (char*)"mag", TLIB_SHM_YES);
    
    if (tt->optSharedI2cFlag) pi2cInit(tt->optI2cPath, tt->optSharedI2cFlag);

    // create mpu connection
    MPU6050 	mpu(tt->optI2cPath, 0x68);

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
    magFd = pi2cOpen(tt->optI2cPath, 0x1e);
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
    
    usleep(100000);

    double mgPerDigit = 0.92f;

    sampleTime = raspilotTlibUsecTime();
    t0 = sampleTime / 1000000.0;
    i = 0;
    for(;;) {
	// read magnetometer
	// pi2cWriteBytesToReg(magFd, 0x03, 0, NULL);
	pi2cReadBytes(magFd, 0x03, 6, mm);
	sampleTime = raspilotTlibUsecTime();
	t1 = sampleTime / 1000000.0;

	MgX = ((int16_t)mm[0] << 8) | mm[1];
	MgY = ((int16_t)mm[2] << 8) | mm[3];
	MgZ = ((int16_t)mm[4] << 8) | mm[5];

	mxyz[0] = MgX;
	mxyz[1] = MgY;
	mxyz[2] = MgZ;
	
	raspilotTlibSend(tt, streami, sampleTime, 1.0, mxyz, 3);
	t0 = t1;
	raspilotTlibMainLoopSleep(tt, sampleTime);
    }

    taskStop(0);
}
