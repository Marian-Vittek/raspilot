
#include "common.h"
#include "Fusion.h"
#include "MPU6050.h"

static void taskStop(int signum) {
    printf("Info: %s exiting\n", __FILE__); fflush(stdout);
    exit(0);
}

int main(int argc, char **argv) {
    double 			t0, t1, samplePeriod;
    int64_t			sampleTime;
    FusionAhrs 			ahrs;
    int				i, streami;
    int16_t 			AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
    double			rpy[3];    
    struct raspilotTlibStr      ttt, *tt;

    tt = raspilotTlibInit(&ttt, argc, argv, TLIB_UNIVERSE_MAP_NO);
    streami = raspilotTlibInitStream(tt, (char*)"rpy", TLIB_SHM_YES);
    
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

    mpu.setDLPFMode(0);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
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
	accelerometer.axis.x = AcX / 16384.0; 
	accelerometer.axis.y = AcY / 16384.0; 
	accelerometer.axis.z = AcZ / 16384.0; 
	gyroscope.axis.x = GyX /  131.0;
	gyroscope.axis.y = GyY / 131.0;
	gyroscope.axis.z = GyZ / 131.0;
	
	sampleTime = raspilotTlibUsecTime();
	t1 = sampleTime / 1000000.0;
	samplePeriod = t1 - t0;
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, samplePeriod);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

	// pitch - negative == nose down;      positive == nose up
	// roll  - negative == left wing down; positive == left wing up
	// yaw   - positive == rotated counterclockwise (view from up)
	rpy[0] = euler.angle.pitch*M_PI/180.0;
	rpy[1] = euler.angle.roll*M_PI/180.0;
	rpy[2] = euler.angle.yaw*M_PI/180.0;

	raspilotTlibSend(tt, streami, sampleTime, 1.0, rpy, 3);

	// The original stuff printed by fusion
	//printf("T:%6.4f: Roll %7.2f, Pitch %7.2f, Yaw %7.2f\n", samplePeriod, euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

	t0 = t1;

	// sleep depending on user defined rate
	raspilotTlibMainLoopSleep(tt, sampleTime);
    }

    taskStop(0);
}
