#include "common.h"
#include "Fusion.h"
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
    int16_t 			AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
    double			rpy[3];    
    struct raspilotTlibStr      ttt, *tt;

    tt = raspilotTlibInit(&ttt, argc, argv, TLIB_UNIVERSE_MAP_NO);
    streami = raspilotTlibInitStream(tt, (char*)"rpy", TLIB_SHM_YES);
    
    if (tt->optSharedI2cFlag) pi2cInit(tt->optI2cPath, tt->optSharedI2cFlag);
    
    // Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, tt->sampleRate);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = (unsigned)(5 * tt->sampleRate), /* was 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    // create mpu connection
    MPU6050 	mpu(tt->optI2cPath, 0x68);

    if (mpu.initialize() != 0) return(-1);

    // This allows magnetometer on mpu6050
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true) ;
    mpu.setSleepEnabled(false);
    //mpu.MPU6050_write_reg (0x6A, 0);
    //mpu.MPU6050_write_reg (0x37, 2);
    //mpu.MPU6050_write_reg (0x6B, 0);
    
    signal(SIGINT, taskStop);
  
    // turn off DLPF, it is only adding latency
    mpu.setDLPFMode(0);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    usleep(1000);
    
    // connect to magnetometer
    magFd = pi2cOpen(tt->optI2cPath, 0x1e);
    if (magFd < 0) {
	fprintf(stderr, "pi2c magnetometer connection failed\n");
	return(-1);
    }
    // 75Hz refresh rate
    pi2cWriteByteToReg(magFd, 0x00, 0x14);
    // continuous mode
    pi2cWriteByteToReg(magFd, 0x02, 0x00);

    usleep(100000);

    sampleTime = raspilotTlibUsecTime();
    t0 = sampleTime / 1000000.0;
    i = 0;
    for(;;) {
        FusionVector gyroscope = {0.0f, 0.0f, 0.0f};     // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {0.0f, 0.0f, 1.0f}; // replace this with actual accelerometer data in g
        FusionVector magnetometer = {1.0f, 0.0f, 0.0f}; // replace this with actual magnetometer data in arbitrary units
	double temp;

	mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
	accelerometer.axis.x = AcX / 16384.0; 
	accelerometer.axis.y = AcY / 16384.0; 
	accelerometer.axis.z = AcZ / 16384.0; 
	gyroscope.axis.x = GyX /  131.0;
	gyroscope.axis.y = GyY / 131.0;
	gyroscope.axis.z = GyZ / 131.0;

	// read magnetometer
	pi2cReadBytes(magFd, 0x03, 6, mm);
	magnetometer.axis.x = ((int16_t)mm[0] << 8) | mm[1];
	magnetometer.axis.y = ((int16_t)mm[2] << 8) | mm[3];
	magnetometer.axis.z = ((int16_t)mm[4] << 8) | mm[5];
	
	sampleTime = raspilotTlibUsecTime();
	t1 = sampleTime / 1000000.0;
	samplePeriod = t1 - t0;

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, samplePeriod);

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

	// pitch - negative == nose down;      positive == nose up
	// roll  - negative == left wing down; positive == left wing up
	// yaw   - positive == rotated counterclockwise (view from up)
	rpy[0] = euler.angle.pitch*M_PI/180.0;
	rpy[1] = euler.angle.roll*M_PI/180.0;
	rpy[2] = euler.angle.yaw*M_PI/180.0;

	raspilotTlibSend(tt, streami, sampleTime, 1.0, rpy, 3);
	t0 = t1;
	raspilotTlibMainLoopSleep(tt, sampleTime);
    }

    taskStop(0);
}
