#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include "pi2c.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// command line options
uint8_t	printRpy = 1;
uint8_t	printQuaternion = 0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

static void setup(int argc, char **argv) {
  int 	i;
  char	*i2cpath;
  int	optSharedI2cFlag;

  i2cpath = (char*)"/dev/i2c-1";
  optSharedI2cFlag = 0;
  
  // TODO: get also i2c device path from command line!
  for(i=1; i<argc; i++) {
    if (strcmp(argv[i], "-rpy") == 0) {
      printRpy = 1;
    } else if (strcmp(argv[i], "--rpy") == 0) {
      printRpy = 0;
    } else if (strcmp(argv[i], "-quat") == 0) {
      printQuaternion = 1;
    } else if (strcmp(argv[i], "--quat") == 0) {
      printQuaternion = 0;
    } else if (strcmp(argv[i], "-s") == 0) {
      // share i2c. Do not reset shared semaphores
      optSharedI2cFlag = 1;
    } else if (argv[i][0] != '-') {
      i2cpath = argv[i];
    }
  }

  if (optSharedI2cFlag) pi2cInit(i2cpath, optSharedI2cFlag);
  
  // initialize device
  printf("Initializing I2C devices...\n");
  // Remove this
  // pi2cInit((char*)"/dev/i2c-1", 1);
  mpu.initialize(i2cpath, 0x68);

  // Is DLPF adding latency also with dmp?
  mpu.setDLPFMode(0);
  usleep(10000);
    
  // verify connection
  printf("Testing device connections...\n");
  printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

  // load and configure the DMP
  printf("Initializing DMP...\n");
  devStatus = mpu.dmpInitialize();

  // how to get those values?
  // gx_offset=-mean_gx/4;
  // gy_offset=-mean_gy/4;
  // gz_offset=-mean_gz/4;

  // 50 10 1
  mpu.setXGyroOffset(-50/4);
  mpu.setYGyroOffset(10/4);
  mpu.setZGyroOffset(-1/4);

  // ax_offset=-mean_ax/8;
  // ay_offset=-mean_ay/8;
  // az_offset=(16384-mean_az)/8;
  // -996 280 16828
  //mpu.setXAccelOffset(); 
  //mpu.setYAccelOffset(); 
  //mpu.setZAccelOffset(); 
    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    printf("Enabling DMP...\n");
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    //attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    printf("DMP ready!\n");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    printf("DMP Initialization failed (code %d)\n", devStatus);
  }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

static int ncount = 0;

static int loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return(0);
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    return(0);
  } else if (fifoCount >= 42) {
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

#if 0
    // I've added this for calibration
    {
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      printf("debug motion6: %d %d %d %d %d %d\n", ax, ay, az, gx, gy, gz);
    }
#endif

    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // original
    //printf("quat %9.7f %9.7f %9.7f %9.7f\n", q.w,q.x,q.y,q.z);

    if (printQuaternion) {
      // translated to drone orientation
      // translation discovered by pure experimentation
#if 0    
      // This is for the following orientation of MPU development board:
      //    front of the drone
      //     ^
      //   o   I
      //       I
      //   o   I
      //holes  pins     
      printf("quat %9.7f %9.7f %9.7f %9.7f\n", -q.y, q.x, q.z, q.w);
#elif 1
      // This is for the following orientation of MPU development board:
      //    front of the drone
      //     ^
      //   I   o
      //   I   
      //   I   o
      // pins holes
      printf("quat %9.7f %9.7f %9.7f %9.7f\n", q.y, -q.x, q.z, q.w);
#endif
    }
    
    if (printRpy) {
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      printf("rpy %9.7f %9.7f %9.7f\n", -ypr[1], ypr[2], -ypr[0]);
    }    
    
    
    fflush(stdout);
    ncount ++;
    return(1);
  }
  return(0);
}

long long getTimeMsec() {
    struct timeval tv;
    
    gettimeofday(&tv, NULL);
    return(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

int main(int argc, char **argv) {
  int r;
  int shortSleepCount;
  int sleepUsec;
  unsigned debugCount;
  long long startTime;

  
  setup(argc, argv);
  usleep(100000);
  
#if DDDEBUG
  setRangePerDigit();
#endif
  
  debugCount = 0;
  shortSleepCount = 0;
  sleepUsec = 3000;
  // for(startTime=getTimeMsec(); getTimeMsec()<startTime+10000; ) {
  for(;;) {
    r = loop();
    if (r) {
      // auto determine for how long I can sleep here
      if (debugCount++ % 100 == 0) {printf("debug sleeps: %d + 200 * %d usec\n", sleepUsec, shortSleepCount); fflush(stdout);}
      sleepUsec = sleepUsec - 200 + shortSleepCount * 200;
      if (sleepUsec <= 200) sleepUsec = 200;
      usleep(sleepUsec);
      shortSleepCount = 0;
    } else {
      usleep(200);
      shortSleepCount ++;
    }
  }	

  printf("Count == %d in %5lld ms.\n", ncount, getTimeMsec()-startTime);

  return 0;
}

