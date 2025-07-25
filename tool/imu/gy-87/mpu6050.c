#include "common.h"
#include "Fusion.h"
#include "MPU6050.h"

static void taskStop(int signum) {
    printf("Info: %s exiting\n", __FILE__); fflush(stdout);
    exit(0);
}

//////////////////////////////////////////////


int main(int argc, char **argv) {
    double 	t0, t1, samplePeriod;
    int64_t	sampleTime;
    FusionAhrs 	ahrs;
    int		i, usleepTime;
    int16_t 	AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
    FusionEuler euler;
    FusionVector facc;
    FusionQuaternion fquat;
    
    int		mpuAccRange, mpuGyroRange;
    double 	accDivider, gyroDivider;
    int		optDLPFilterMode;
    double	rpy[3];    
    double	acc[3];    
    double	eacc[3];    
    double      quat[4];
    int 	streamirpy, streamieacc;
    
    struct raspilotTlibStr      ttt, *tt;

    optDLPFilterMode = 0;		// default - no filter
    
    for(i=1; i<argc; i++) {
	if (strcmp(argv[i], "-f") == 0) {
	    // DLPFilterMode
	    i++;
	    if (i<argc) optDLPFilterMode = strtol(argv[i], NULL, 10);
	}
    }	

    tt = raspilotTlibInit(&ttt, argc, argv, TLIB_UNIVERSE_MAP_NO);
    streamirpy = raspilotTlibInitStream(tt, (char*)"rpy", TLIB_SHM_YES);
    streamieacc = raspilotTlibInitStream(tt, (char*)"eacc", TLIB_SHM_YES);
    
    if (tt->optSharedI2cFlag) pi2cInit(tt->optI2cPath, tt->optSharedI2cFlag);

    // create mpu connection
    MPU6050 	mpu(tt->optI2cPath, 0x68);

    if (mpu.initialize() != 0) return(-1);

    // This allows magnetometer on mpu6050
    // MPU6050_write_reg (0x6A, 0);
    // MPU6050_write_reg (0x37, 2);
    // MPU6050_write_reg (0x6B, 0);
    
    signal(SIGINT, taskStop);
  
    // turn off DLPF, it is only adding latency
    // printf("Info %s:%d\n", __FILE__, __LINE__); fflush(stdout);

    // Set to the one working on your model in the range 0-6
    mpu.setDLPFMode(optDLPFilterMode);

    mpuGyroRange = MPU6050_GYRO_FS_1000;
    mpu.setFullScaleGyroRange(mpuGyroRange);
    gyroDivider = 131.0 / (1<<mpuGyroRange);
	
    //AFS_SEL | Full Scale Range | LSB Sensitivity
    //--------+------------------+----------------
    //0 | +/- 2g | 16384 LSB/g
    //1 | +/- 4g | 8192 LSB/g
    //2 | +/- 8g | 4096 LSB/g
    //3 | +/- 16g | 2048 LSB/g
    mpuAccRange = MPU6050_ACCEL_FS_8;
    mpu.setFullScaleAccelRange(mpuAccRange);
    accDivider = 16384.0 / (1<<mpuAccRange);
    
    FusionAhrsInitialise(&ahrs);
    
    usleep(100000);

    sampleTime = raspilotTlibUsecTime();
    t0 = sampleTime / 1000000.0;
    i = 0;
    for(;;) {
        FusionVector gyroscope = {0.0f, 0.0f, 0.0f};     // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {0.0f, 0.0f, 1.0f}; // replace this with actual accelerometer data in g
	double temp;

	mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
	accelerometer.axis.x = AcX / accDivider;
	accelerometer.axis.y = AcY / accDivider;
	accelerometer.axis.z = AcZ / accDivider;
	gyroscope.axis.x = GyX /  gyroDivider;
	gyroscope.axis.y = GyY / gyroDivider;
	gyroscope.axis.z = GyZ / gyroDivider;

#if 0
	// This is a strange ad-hoc adjustement. It seems that my mpu6050 reports
	// values on exponential scale instead of linear. Is it even possible?
	accelerometer.axis.x = rescaleAcceleration(accelerometer.axis.x);
	accelerometer.axis.y = rescaleAcceleration(accelerometer.axis.y);
	accelerometer.axis.z = rescaleAcceleration(accelerometer.axis.z);
#endif
	
	sampleTime = raspilotTlibUsecTime();
	t1 = sampleTime / 1000000.0;
	samplePeriod = t1 - t0;
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, samplePeriod);

	fquat = FusionAhrsGetQuaternion(&ahrs);

	quat[0] = fquat.element.x;
	quat[1] = fquat.element.y;
	quat[2] = fquat.element.z;
	quat[3] = fquat.element.w;

        euler = FusionQuaternionToEuler(fquat);
	// roll  - negative == left wing down; positive == left wing up
	// pitch - negative == nose down;      positive == nose up
	// yaw   - positive == rotated counterclockwise (view from up)
	rpy[0] = euler.angle.roll*M_PI/180.0;
	rpy[1] = euler.angle.pitch*M_PI/180.0;
	rpy[2] = euler.angle.yaw*M_PI/180.0;
	
	acc[0] = accelerometer.axis.x;
	acc[1] = accelerometer.axis.y;
	acc[2] = accelerometer.axis.z;

	facc = FusionAhrsGetEarthAcceleration(&ahrs);
	eacc[0] = facc.axis.x;
	eacc[1] = facc.axis.y;
	eacc[2] = facc.axis.z;
	
	// printf("debug: accelerometer.axis.z = mpu: %5.3f; earth:  %5.3f; \n", accelerometer.axis.z, facc.axis.z);fflush(stdout);
	FusionAhrsGetQuaternion(&ahrs);

	raspilotTlibSend(tt, streamirpy, sampleTime, 1.0, rpy, 3);
	raspilotTlibSend(tt, streamieacc, sampleTime, 1.0, eacc, 3);
	// raspilotTlibSend(tt, streamiacc, sampleTime, 1.0, acc, 3);
	// raspilotTlibSend(tt, streamiquat, sampleTime, 1.0, quat, 4);

	// The original stuff printed by fusion
	//printf("T:%6.4f: Roll %7.2f, Pitch %7.2f, Yaw %7.2f\n", samplePeriod, euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
	t0 = t1;
	raspilotTlibMainLoopSleep(tt, sampleTime);
    }

    taskStop(0);
}
