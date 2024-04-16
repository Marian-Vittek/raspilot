#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <signal.h>

#include "Fusion.h"
#include "pi2c.h"
#include "MPU6050.h"

#ifdef SHM
#include "raspilotshm.h"
#endif

static inline double doubleGetTime() {
  struct timespec tt;
  clock_gettime(CLOCK_REALTIME, &tt);
  return(tt.tv_sec + tt.tv_nsec/1000000000.0);
}

static void taskStop(int signum) {
    printf("Info: %s exiting\n", __FILE__); fflush(stdout);
    exit(0);
}

int main(int argc, char **argv) {
    double 	t0, t1, samplePeriod;
    FusionAhrs 	ahrs;
    int		i, usleepTime;
    int16_t 	AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

    int		optSharedI2cFlag;
    char	*optI2cPath;
    double	optRate;
    double	rpy[3];    

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

    // This allows magnetometer on mpu6050
    // MPU6050_write_reg (0x6A, 0);
    // MPU6050_write_reg (0x37, 2);
    // MPU6050_write_reg (0x6B, 0);
    
#ifdef SHM
    shmbuf = raspilotShmConnect((char *)"raspilot.gyro-mpu6050-magwick-shm.rpy");
    shmbuf2 = raspilotShmConnect((char *)"raspilot.gyro-mpu6050-magwick-shm.rpy2");
    if (shmbuf == NULL) exit(-1);
#endif
    
    signal(SIGINT, taskStop);
  
    // turn off DLPF, it is only adding latency
    // printf("Info %s:%d\n", __FILE__, __LINE__); fflush(stdout);

    mpu.setDLPFMode(0);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    FusionAhrsInitialise(&ahrs);
    
    usleepTime = 1000000 / optRate;
    usleep(usleepTime);

    t0 = doubleGetTime();
    i = 0;
    for(;;) {
        FusionVector gyroscope = {0.0f, 0.0f, 0.0f};     // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {0.0f, 0.0f, 1.0f}; // replace this with actual accelerometer data in g
	double temp;

	mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
	accelerometer.axis.x = AcX / 16384.0; 
	accelerometer.axis.y = AcY / 16384.0; 
	accelerometer.axis.z = AcZ / 16384.0; 
	gyroscope.axis.x = GyX /  131.0;
	gyroscope.axis.y = GyY / 131.0;
	gyroscope.axis.z = GyZ / 131.0;
	
	t1 = doubleGetTime();
	samplePeriod = t1 - t0;
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, samplePeriod);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

	// pitch - negative == nose down;      positive == nose up
	// roll  - negative == left wing down; positive == left wing up
	// yaw   - positive == rotated counterclockwise (view from up)
	rpy[0] = euler.angle.pitch*M_PI/180.0;
	rpy[1] = euler.angle.roll*M_PI/180.0;
	rpy[2] = euler.angle.yaw*M_PI/180.0;
	
#ifdef SHM
	shmbuf->confidence = 1.0;
	if (raspilotShmPush(shmbuf, t1, rpy, 3) != 0) taskStop(0);
	if (shmbuf2 != NULL) if (raspilotShmPush(shmbuf2, t1, rpy, 3) != 0) taskStop(0);
	// printf("debug: rpy %g: %9.7f %9.7f %9.7f\n", t1, rpy[0], rpy[1], rpy[2]);
#else
	printf("rpy %9.7f %9.7f %9.7f\n", rpy[0], rpy[1], rpy[2]);
	fflush(stdout);
#endif	
	// The original stuff printed by fusion
	//printf("T:%6.4f: Roll %7.2f, Pitch %7.2f, Yaw %7.2f\n", samplePeriod, euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

	t0 = t1;

	if (samplePeriod > 1.0/optRate && usleepTime > 0) usleepTime--;
	else if (samplePeriod < 1.0/optRate) usleepTime++;
	usleep(usleepTime);

	// if (i++ % 1000 == 0) printf("debug usleepTime == %d\n", usleepTime);
    }

    taskStop(0);
}
