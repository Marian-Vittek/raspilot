#include "common.h"
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


static void taskStop(int signum) {
    exit(0);
}

int main(int argc, char **argv) {
    double 			t0, t1, samplePeriod;
    int64_t			sampleTime;
    int				i, r, streami;
    int				magFd;
    uint8_t			mm[6];
    int16_t 			MgX,MgY,MgZ;

    char			*calibrationFile;
    double			mxyz[3];
    struct raspilotTlibStr      ttt, *tt;


    tt = raspilotTlibInit(&ttt, argc, argv, TLIB_UNIVERSE_MAP_NO);
    streami = raspilotTlibInitStream(tt, (char*)"mag", TLIB_SHM_YES);
    
    if (tt->optSharedI2cFlag) pi2cInit(tt->optI2cPath, tt->optSharedI2cFlag);

    // create mpu connection
    MPU6050 	mpu(tt->optI2cPath, 0x68);

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
    magFd = pi2cOpen(tt->optI2cPath, 0x0d);
    if (magFd < 0) {
	fprintf(stderr, "pi2c magnetometer connection failed\n");
	return(-1);
    }
    
    pi2cWriteByteToReg(magFd, QMC5883L_RESET, 0x01);
    pi2cWriteByteToReg(magFd, QMC5883L_CONFIG,  QMC5883L_CONFIG_OS512 | QMC5883L_CONFIG_2GAUSS | QMC5883L_CONFIG_50HZ | QMC5883L_CONFIG_CONT);

    usleep(100000);

    sampleTime = raspilotTlibUsecTime();
    t0 = sampleTime / 1000000.0;
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
	sampleTime = raspilotTlibUsecTime();
	t1 = sampleTime / 1000000.0;
	MgX = ((int16_t)mm[1] << 8) | mm[0];
	MgY = ((int16_t)mm[3] << 8) | mm[2];
	MgZ = ((int16_t)mm[5] << 8) | mm[4];

	// return as roll, pitch, yaw. Raspilot shall be configured in the way that he knows that only yaw is valid.
	mxyz[0] = MgX;
	mxyz[1] = MgY;
	mxyz[2] = MgZ;
	
	raspilotTlibSend(tt, streami, sampleTime, 1.0, mxyz, 3);
	t0 = t1;
	raspilotTlibMainLoopSleep(tt, sampleTime);
    }

    taskStop(0);
}


