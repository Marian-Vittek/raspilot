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

#include "common.h"
#include "Fusion.h"
#include "DFRobot_BMI160.h"

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

static void taskStop(int signum) {
    exit(0);
}

int main(int argc, char **argv) {
    double 			t0, t1, samplePeriod;
    int64_t			sampleTime;
    FusionAhrs 			ahrs;
    int				i, streami;
    int16_t 			AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
    int 			rslt;
    int16_t 			accelGyro[6]={0}; 
    int 			ii;
    double			rpy[3];    
    struct raspilotTlibStr      ttt, *tt;


    tt = raspilotTlibInit(&ttt, argc, argv, TLIB_UNIVERSE_MAP_NO);
    streami = raspilotTlibInitStream(tt, (char*)"rpy", TLIB_SHM_YES);
    
    if (tt->optSharedI2cFlag) pi2cInit(tt->optI2cPath, tt->optSharedI2cFlag);

    if (bmi160.I2cInit(tt->optI2cPath, i2c_addr) != BMI160_OK){
	fprintf(stderr, "%s:%d: Can't init bmi160\n", __FILE__, __LINE__);
	exit(-1);
    }
    bmi160.setStepPowerMode(bmi160.stepNormalPowerMode);

    // reset gyroscope sensibility
    bmi160.defaultParamSettg(&dev);
    dev.gyroCfg.range = BMI160_GYRO_RANGE_500_DPS; // BMI160_GYRO_RANGE_2000_DPS;
    bmi160.setGyroConf(&dev);
    
    signal(SIGINT, taskStop);
  
    FusionAhrsInitialise(&ahrs);
    
    usleep(100000);

    sampleTime = raspilotTlibUsecTime();
    t0 = sampleTime / 1000000.0;
    i = ii = 0;
    for(;;) {
        FusionVector gyroscope = {0.0f, 0.0f, 0.0f};     // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {0.0f, 0.0f, 1.0f}; // replace this with actual accelerometer data in g
	double temp;

	// get both accel and gyro data from bmi160
	// parameter accelGyro is the pointer to store the data
	rslt = bmi160.getAccelGyroData(accelGyro);
	sampleTime = raspilotTlibUsecTime();
	t1 = sampleTime / 1000000.0;
	samplePeriod = t1 - t0;
	
	if(rslt != 0) {
	    printf("debug Error getting data\n");
	} else {
	    gyroscope.axis.x = accelGyro[0] / FACTOR_GYRO;
	    gyroscope.axis.y = accelGyro[1] / FACTOR_GYRO;
	    gyroscope.axis.z = accelGyro[2] / FACTOR_GYRO;
	    accelerometer.axis.x = accelGyro[3] / FACTOR_ACC;
	    accelerometer.axis.y = accelGyro[4] / FACTOR_ACC;
	    accelerometer.axis.z = accelGyro[5] / FACTOR_ACC;

	    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, samplePeriod);
	    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

	    // roll  - negative == left wing down; positive == left wing up
	    // pitch - negative == nose down;      positive == nose up
	    // yaw   - positive == rotated counterclockwise (view from up)
	    rpy[0] = euler.angle.roll*M_PI/180.0;
	    rpy[1] = euler.angle.pitch*M_PI/180.0;
	    rpy[2] = euler.angle.yaw*M_PI/180.0;
	
	    raspilotTlibSend(tt, streami, sampleTime, 1.0, rpy, 3);
	}

	t0 = t1;
	raspilotTlibMainLoopSleep(tt, sampleTime);
    }

    taskStop(0);
}
