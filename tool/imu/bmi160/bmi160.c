/*!
 * @file accelGyro.ino
 * @brief I2C addr:
 * @n  0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
 * @n  0x69: set I2C address by parameter
 * @n Through the example, you can get the sensor data by using getSensorData:
 * @n get acell by paremeter onlyAccel;
 * @n get gyro by paremeter onlyGyro;
 * @n get both acell and gyro by paremeter bothAccelGyro.
 * @n With the rotation of the sensor, data changes are visible.
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author  DFRobot_haoJ(hao.jiang@dfrobot.com)
 * @version V1.0
 * @date 2017-12-01
 * @url https://github.com/DFRobot/DFRobot_BMI160
 */

/*
  (C) Marian Vittek adjusted for Raspberry Pi and Madgwick Fusion.
 */

#include <time.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>

#include "Fusion.h"
#include "DFRobot_BMI160.h"

#ifdef SHM
#include "raspilotshm.h"
#endif

#define FACTOR_GYRO_RFS2000 16.4
#define FACTOR_GYRO_RFS1000 32.8
#define FACTOR_GYRO_RFS500 65.6
#define FACTOR_GYRO_RFS250 131.2
#define FACTOR_GYRO_RFS125 262.4

#define FACTOR_ACC_S2g  16384.0
#define FACTOR_ACC_S4g  8192.0
#define FACTOR_ACC_S8g  4096.0
#define FACTOR_ACC_S16g 2048.0
    
#define FACTOR_GYRO FACTOR_GYRO_RFS1000
#define FACTOR_ACC  FACTOR_ACC_S2g


DFRobot_BMI160 bmi160;
struct bmi160Dev dev;
const int8_t i2c_addr = 0x69;

static inline double doubleGetTime() {
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    return(tv.tv_sec + tv.tv_usec / 1000000.0);
}
#if 0
static inline double doubleGetTime() {
  struct timespec tt;
  clock_gettime(CLOCK_REALTIME, &tt);
  return(tt.tv_sec + tt.tv_nsec/1000000000.0);
}
#endif

static void taskStop(int signum) {
    exit(0);
}

int main(int argc, char **argv) {
    double 	t0, t1, samplePeriod;
    FusionAhrs 	ahrs;
    int		i, usleepTime;
    int16_t 	AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
    int 	rslt;
    int16_t 	accelGyro[6]={0}; 
    int 	ii;
    double	rpy[3];    
    int		optSharedI2cFlag;
    char	*optI2cPath;
    double	optRate;
#ifdef SHM
    struct raspilotInputBuffer 	*shmbuf;
    struct raspilotInputBuffer 	*shmbuf2;
#endif

    optSharedI2cFlag = 0;
    optI2cPath = (char*)"/dev/i2c-1";
    optRate = 1000.0; 			// default rate 1kHz
    
    for(i=1; i<argc; i++) {
	if (strcmp(argv[i], "-s") == 0) {
	    // share i2c. Do not reset shared semaphores
	    optSharedI2cFlag = 1;
	} else if (strcmp(argv[i], "-r") == 0) {
	    // refresh rate in Hz, reasonable values are up to 3000Hz.
	    i++;
	    if (i<argc) optRate = strtod(argv[i], NULL);
	} else {
	    optI2cPath = argv[i];
	}
    }	
    
    if (optSharedI2cFlag) pi2cInit(optI2cPath, optSharedI2cFlag);

    if (bmi160.I2cInit((char*)"/dev/i2c-1", i2c_addr) != BMI160_OK){
	fprintf(stderr, "%s:%d: Can't init bmi160\n", __FILE__, __LINE__);
	exit(-1);
    }
    bmi160.setStepPowerMode(bmi160.stepNormalPowerMode);

    // reset gyroscope sensibility
    bmi160.defaultParamSettg(&dev);
    dev.gyroCfg.range = BMI160_GYRO_RANGE_500_DPS; // BMI160_GYRO_RANGE_2000_DPS;
    bmi160.setGyroConf(&dev);
    

#ifdef SHM
    shmbuf = raspilotShmConnect((char *)"raspilot.gyro-bmi-magwick-shm.rpy");
    shmbuf2 = raspilotShmConnect((char *)"raspilot.gyro-bmi-magwick-shm.rpy2");
    if (shmbuf == NULL) exit(-1);
#endif
    
    signal(SIGINT, taskStop);
  
    FusionAhrsInitialise(&ahrs);
    
    usleepTime = 1000000 / optRate;
    usleep(usleepTime);

    t0 = doubleGetTime();
    i = ii = 0;
    for(;;) {
        FusionVector gyroscope = {0.0f, 0.0f, 0.0f};     // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {0.0f, 0.0f, 1.0f}; // replace this with actual accelerometer data in g
	double temp;

	// get both accel and gyro data from bmi160
	// parameter accelGyro is the pointer to store the data
	rslt = bmi160.getAccelGyroData(accelGyro);
	if(rslt != 0) {
	    printf("debug Error getting data\n");
	} else {
	    gyroscope.axis.x = accelGyro[0] / FACTOR_GYRO;
	    gyroscope.axis.y = accelGyro[1] / FACTOR_GYRO;
	    gyroscope.axis.z = accelGyro[2] / FACTOR_GYRO;
	    accelerometer.axis.x = accelGyro[3] / FACTOR_ACC;
	    accelerometer.axis.y = accelGyro[4] / FACTOR_ACC;
	    accelerometer.axis.z = accelGyro[5] / FACTOR_ACC;

	    t1 = doubleGetTime();
	    samplePeriod = t1 - t0;
	    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, samplePeriod);
	    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

	    // Print rpy in drone coordinates. This depends on how precisely the sensor is mounted on drone.
	    // TODO: Maybe print in sensor's coordinates and make translation inside raspilot.
	    // pitch - negative == nose down;      positive == nose up
	    // roll  - negative == left wing down; positive == left wing up
	    // yaw   - positive == rotated counterclockwise (view from up)
	    rpy[0] = -euler.angle.pitch*M_PI/180.0;
	    rpy[1] = -euler.angle.roll*M_PI/180.0;
	    rpy[2] = euler.angle.yaw*M_PI/180.0;
#ifdef SHM
	    shmbuf->confidence = 1.0;
	    if (raspilotShmPush(shmbuf, t1, rpy, 3) != 0) exit(0);
	    if (shmbuf2 != NULL) if (raspilotShmPush(shmbuf2, t1, rpy, 3) != 0) exit(0);
#else
	    printf("rpy %9.7f %9.7f %9.7f\n", rpy[0], rpy[1], rpy[2]);
	    fflush(stdout);
#endif		
	    // The original stuff printed by fusion
	    // printf("T:%6.4f: Roll %7.2f, Pitch %7.2f, Yaw %7.2f\n", samplePeriod, euler.angle.roll, euler.angle.pitch, euler.angle.yaw); fflush(stdout);
	}

	t0 = t1;
	if (samplePeriod > 1.0/optRate && usleepTime > 0) usleepTime--;
	else if (samplePeriod < 1.0/optRate) usleepTime++;
	usleep(usleepTime);

	// if (i++ % 1000 == 0) printf("debug usleepTime == %d\n", usleepTime);
    }

    taskStop(0);
}
