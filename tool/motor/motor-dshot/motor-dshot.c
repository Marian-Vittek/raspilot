/*
Code to generate DSHOT protocol signals from a Raspberry Pi GPIO
output.  Inspired by https://github.com/dmrlawson/raspberrypi-dshot.
This code uses 'clock_gettime' for timing and allows to broadcast
mutliple dshot frames on multiple pins at once.  "getGpioRegBase"
function was taken from
https://stackoverflow.com/questions/69425540/execute-mmap-on-linux-kernel
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <assert.h>
#include <stdint.h>
#include <sched.h>
#include <errno.h>
#include <string.h>

// #include "../motor-common.h"

//////////////////////////////////////////////////////////////////////
// Select the dshot version you want to use. Value may be 150, 300,
// 600 or 1200. DSHOT_VERSION 150 seems to work fine on raspberry pi
// zero 2, other values may not work.
#ifndef DSHOT_VERSION
#define DSHOT_VERSION 150
#endif


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// DSHOT_BIT_ns specifies the length of 1 bit in nanoseconds for
// various Dshot protocols.

#if DSHOT_VERSION == 150

// DSHOT_150
// Theoretical value from protocol definition
//#define DSHOT_BIT_ns 6670
// Value for my cheap ESC
#define DSHOT_BIT_ns 6000

#elif DSHOT_VERSION == 300

// DSHOT_300
// Theoretical values from protocol definition
//#define DSHOT_BIT_ns 3330
// Value for my cheap ESC
#define DSHOT_BIT_ns 2700

#elif DSHOT_VERSION == 600

// DSHOT_600
// Not tested
#define DSHOT_BIT_ns 1670

#elif DSHOT_VERSION == 1200

// DSHOT_1200
// Not tested
#define DSHOT_BIT_ns 830

#endif


// Define the length of T0H depending on the bit-rate
// Theoretical value from protocol definition
//#define DSHOT_T0H_ns (DSHOT_BIT_ns * 3 / 8)
// Real value for my cheap ESC
#define DSHOT_T0H_ns (DSHOT_BIT_ns * 3 / 9)

// Which clocks to use for timing. Standard constants CLOCK_... from
// time.h can be used here.
#ifndef DSHOT_USE_CLOCK
#define DSHOT_USE_CLOCK     	CLOCK_MONOTONIC_RAW
#endif

#define DSHOT_MAX_TIMING_ERROR_ns       2000
// how many times we retry to send dshot frame if previous was not timed well
#define DSHOT_MAX_RETRY                 10
#define USLEEP_BEFORE_REBROADCAST       100


#define TIMESPEC_TO_INT(tt)             (tt.tv_sec * 1000000000LL + tt.tv_nsec)
/////////

#define GPIO_BASE_OFFSET        0x00200000
#define PAGE_SIZE               (4*1024)
#define BLOCK_SIZE              (4*1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g)             (*(dshotGpio+((g)/10)) &= ~(7<<(((g)%10)*3)))
#define OUT_GPIO(g)             (*(dshotGpio+((g)/10)) |=  (1<<(((g)%10)*3)))

#define GPIO_SET                (*(dshotGpio+7))
#define GPIO_CLR                (*(dshotGpio+10))

#define DSHOT_NUM_PINS                27

enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
};

// I/O access
static void                     *dshotGpioMap;
static volatile uint32_t        *dshotGpio;

// 3D mode, if dshot3dMode != 0 then reverse rotation is enabled
static int dshot3dMode = 0;


//////////////////////////////////////////////////////////////////////////////////////


static inline uint64_t dshotGetNanoseconds() {
    struct timespec tt;
    clock_gettime(DSHOT_USE_CLOCK, &tt);
    return(TIMESPEC_TO_INT(tt));
}


static int dshotAddChecksumAndTelemetry(int packet, int telem) {
    int packet_telemetry = (packet << 1) | (telem & 1);
    int i;
    int csum = 0;
    int csum_data = packet_telemetry;
    for (i = 0; i < 3; i++) {
        csum ^=  csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    return ((packet_telemetry << 4) | csum);
}

static void dshotSend(uint32_t allMotorsPinMask, uint32_t *clearMasks) {
    int                 i, j;
    int64_t             t, tt, t0, offset1, offset2, offset3;
    volatile unsigned   *gpioset;
    volatile unsigned   *gpioclear;

    // prepare addresses
    gpioset = &GPIO_SET;
    gpioclear = &GPIO_CLR;


    offset1 = DSHOT_T0H_ns;
    offset2 = DSHOT_T0H_ns;
    offset3 = DSHOT_BIT_ns - offset1 - offset2;

    // We will try to send the frame several times if timing was wrong
    for(j=0; j<DSHOT_MAX_RETRY; j++) {
        // send dshot frame bits
        tt = t0 = dshotGetNanoseconds();
        for(i=0; i<16; i++) {
            tt += offset3;
            while ((t=dshotGetNanoseconds()) < tt) ;
            *gpioset = allMotorsPinMask;
            // if we are not in time, abandon the whole dshot frame
            if (t - tt > DSHOT_MAX_TIMING_ERROR_ns) break;
            tt += offset1;
            while ((t=dshotGetNanoseconds()) < tt) ;
            *gpioclear = clearMasks[i];
            if (t - tt > DSHOT_MAX_TIMING_ERROR_ns) break;
            tt += offset2;
            while ((t=dshotGetNanoseconds()) < tt) ;
            *gpioclear = allMotorsPinMask;
            if (t - tt > DSHOT_MAX_TIMING_ERROR_ns) break;
        }
        if (t - tt > DSHOT_MAX_TIMING_ERROR_ns) {
            // we were out of timing, the frame was abandonned and we will retry to broadcast it
            *gpioclear = allMotorsPinMask;
            // printf("debug Dshot Frame was abandonned because of wrong timing in attempt %d, bit %d.\n", j, i); fflush(stdout);
            // relax to OS for a small period of time, hope it reduces the probability that we will
            // be interrupted during next broadcasting.
            usleep(USLEEP_BEFORE_REBROADCAST);  
        } else {
            // ok we are done here.
            // printf("debug Dshot Frame successfully sent.\n"); fflush(stdout);
            return;
        }
    }
    printf("debug Dshot Frame failure.\n"); fflush(stdout);
}

static uint32_t dshotGetAllMotorsPinMask(int motorPins[], int motorMax) {
    int         i;
    uint32_t    allMotorsPinsMask;

    // compute masks
    allMotorsPinsMask = 0;
    for(i=0; i<motorMax; i++) allMotorsPinsMask |= (1<<motorPins[i]);
    return(allMotorsPinsMask);
}

void dshotSendFrames(int motorPins[], int motorMax, unsigned frame[]) {
    int         i, bi;
    unsigned    bit;
    uint32_t    clearMasks[16];
    uint32_t    msk, allMotorsMask;

    assert(motorMax < DSHOT_NUM_PINS);

    allMotorsMask = dshotGetAllMotorsPinMask(motorPins, motorMax);

    // compute masks for zero bits in all frames
    for(bi=0; bi<16; bi++) {
        msk = 0;
        bit = (0x8000 >> bi);
        for(i=0; i<motorMax; i++) {
            if ((frame[i] & bit) == 0) msk |= (1<<motorPins[i]);
        }
        clearMasks[bi] = msk;
    }

    dshotSend(allMotorsMask, clearMasks);
}

static uint32_t getGpioRegBase(void) {
    const char *revision_file = "/proc/device-tree/system/linux,revision";
    uint8_t revision[4] = { 0 };
    uint32_t cpu = 4;
    FILE *fd;

    if ((fd = fopen(revision_file, "rb")) == NULL) {
        printf("debug Error: Can't open '%s'\n", revision_file);
    } else {
        if (fread(revision, 1, sizeof(revision), fd) == 4) {
            cpu = (revision[2] >> 4) & 0xf;
        } else {
            printf("debug Error: Revision data too short\n");
        }
        fclose(fd);
    }

    // printf("debug CPU type: %d\n", cpu);
    switch (cpu) {
        case 0: // BCM2835 [Pi 1 A; Pi 1 B; Pi 1 B+; Pi Zero; Pi Zero W]
            return(0x20000000 + GPIO_BASE_OFFSET);
        case 1: // BCM2836 [Pi 2 B]
        case 2: // BCM2837 [Pi 3 B; Pi 3 B+; Pi 3 A+]
            return(0x3f000000 + GPIO_BASE_OFFSET);
        case 3: // BCM2711 [Pi 4 B]
            return(0xfe000000 + GPIO_BASE_OFFSET);
        default:
            printf("debug Error: Unrecognised revision code\n");
            return(0xfe000000 + GPIO_BASE_OFFSET);
    }
}

static void dshotSetupIo() {
    int         mem_fd;
    int32_t     gpioBase;

    gpioBase = getGpioRegBase();
    
    /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("debug Error: Can't open /dev/mem, run using sudo! \n");
      exit(-1);
   }

   /* mmap GPIO */
   dshotGpioMap = mmap(
      NULL,             // Any adddress in our space will do
      BLOCK_SIZE,       // Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       // Shared with other processes
      mem_fd,           // File to map
      gpioBase  // Offset to GPIO peripheral
   );

   close(mem_fd); //No need to keep mem_fd open after mmap

   if (dshotGpioMap == MAP_FAILED) {
      printf("debug Mmap error %p\n", dshotGpioMap);//errno also set!
      exit(-1);
   }

   // Always use volatile pointer!
   dshotGpio = (volatile unsigned *)dshotGpioMap;

}

// Send a command repeatedly during a given perion of time
static void dshotRepeatSendCommand(int motorPins[], int motorMax, int cmd, int telemetry, int timePeriodMsec) {
    unsigned    frame[DSHOT_NUM_PINS+1];
    int         i;
    int64_t     t;
    
    for(i=0; i<motorMax; i++) frame[i] = dshotAddChecksumAndTelemetry(cmd, telemetry);
    t = dshotGetNanoseconds() + timePeriodMsec * 1000000LL;
    while (dshotGetNanoseconds() <= t) {
        dshotSendFrames(motorPins, motorMax, frame);
        usleep(USLEEP_BEFORE_REBROADCAST);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Main exported functions of the module implementing raspilot motor instance.
//////////////////////////////////////////////////////////////////////////////////////////////

// This function allows to set bidirectional rotation (mode3dFlag!=0) and reverse rotation logic (reverseDirectionFlag!=0).
// Changing 3D mode is interfering with rotation direction (at least on my ESC), so always reset the direction when changing 3D.
void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) {
    int         repeatMsec;

    // Repeat the command for some time to take effect. Do it longer in our case as we are not sure to send 
    // all the frames correctly. If repeatMsec == 25, then this function will take 4*25 msec.
    repeatMsec = 25;
    
    dshot3dMode = mode3dFlag;
    if (dshot3dMode) {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_3D_MODE_ON, 1, repeatMsec);
    } else {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_3D_MODE_OFF, 1, repeatMsec);
    }
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SAVE_SETTINGS, 0, repeatMsec);

    if (reverseDirectionFlag) {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SPIN_DIRECTION_REVERSED, 1, repeatMsec);
    } else {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SPIN_DIRECTION_NORMAL, 1, repeatMsec);
    }
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SAVE_SETTINGS, 0, repeatMsec);
}

void motorImplementationInitialize(int motorPins[], int motorMax) {
    int         i, pin;

    dshotSetupIo();
    
    for(i=0; i<motorMax; i++) {
        pin = motorPins[i];
        INP_GPIO(pin);          // must use INP_GPIO before we can use OUT_GPIO
        OUT_GPIO(pin);
        GPIO_CLR = 1<<pin;
    }

    // Maybe by default set to normal direction and no reverse spin
    // motorImplementationSet3dModeAndSpinDirection(motorPins, motorMax, 0, 0);
    // Arm motors by sending DSHOT_CMD_MOTOR_STOP (aka 0) for 5 seconds
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_MOTOR_STOP, 0, 5000);
}

void motorImplementationFinalize(int motorPins[], int motorMax) {
    munmap(dshotGpioMap, BLOCK_SIZE);
}

void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) {
    int         i;
    unsigned    frame[DSHOT_NUM_PINS+1];
    int         val;

    assert(motorMax < DSHOT_NUM_PINS);

    for(i=0; i<motorMax; i++) {
        if (dshot3dMode) {
            // translate double throttles ranging <-1, 1> to dshot frames.
            if (motorThrottle[i] >= 0) {
                val = motorThrottle[i] * 999 + 1048;
            } else {
                val = -motorThrottle[i] * 999 + 48;
            }
        } else {
            // translate double throttles ranging <0, 1> to dshot frames.
            val = motorThrottle[i] * 1999 + 48;
        }
        // we used command 0 for zero thrust which should be used as arming sequence as well.
	// but in 3d mode we have to be carefull it seems to reset the motor.
        if (/*motorThrottle[i] == 0 || */ val < 48 || val >= 2048) val = DSHOT_CMD_MOTOR_STOP;
        frame[i] = dshotAddChecksumAndTelemetry(val, 0);
    }

    dshotSendFrames(motorPins, motorMax, frame);
}


