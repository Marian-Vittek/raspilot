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
#define DSHOT_VERSION 150

// This ad-hoc value have to be "guessed".  It specifies how much in
// advance we are going to clear zero bits in dshot frames. Too small
// value makes ESC to interpret 0 bits as being 1. Too large will
// makes ESC not to recognize the protocol.
#define DSHOT_AD_HOC_OFFSET	(300)




//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


#if DSHOT_VERSION == 150

// DSHOT_150
#define DSHOT_T0H_ns 2500
#define DSHOT_BIT_ns 6670

#elif DSHOT_VERSION == 300

// DSHOT_300
#define DSHOT_T0H_ns 1250
#define DSHOT_BIT_ns 3330

#elif DSHOT_VERSION == 600

// DSHOT_600
#define DSHOT_T0H_ns 0625
#define DSHOT_BIT_ns 1670

#elif DSHOT_VERSION == 1200

// DSHOT_1200
#define DSHOT_T0H_ns 313
#define DSHOT_BIT_ns 830

#endif

//#define DSHOT_USE__CLOCK		CLOCK_REALTIME
#define DSHOT_USE__CLOCK		CLOCK_MONOTONIC_RAW 
#define USLEEP_BEFORE_BROADCAST		100
#define TIMESPEC_TO_INT(tt) 		(tt.tv_sec * 1000000000LL + tt.tv_nsec)

/////////

#define GPIO_BASE_OFFSET 	0x00200000
#define PAGE_SIZE 		(4*1024)
#define BLOCK_SIZE 		(4*1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) 		(*(dshotGpio+((g)/10)) &= ~(7<<(((g)%10)*3)))
#define OUT_GPIO(g) 		(*(dshotGpio+((g)/10)) |=  (1<<(((g)%10)*3)))

#define GPIO_SET 		(*(dshotGpio+7))
#define GPIO_CLR 		(*(dshotGpio+10))

#define NUM_PINS 		27

    
// I/O access
static void 			*dshotGpioMap;
static volatile uint32_t 	*dshotGpio;


static inline uint64_t dshotGetNanoseconds() {
#if 0 && defined(__ARM_ARCH)
  unsigned cc;
  asm volatile ("mrc p15, 0, %0, c15, c12, 1" : "=r" (cc));
  // ??? how to translate to nanoseconds?
  return (cc / ticksPerNanosecond);
#else
  struct timespec tt;
  clock_gettime(DSHOT_USE__CLOCK, &tt);
  return(TIMESPEC_TO_INT(tt));
#endif
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

static void dshotSendFrames(uint32_t allMotorsPinMask, uint32_t *clearMasks) {
    int 		i;
    int64_t		t, offset1, offset2, offset3;
    volatile unsigned	*gpioset;
    volatile unsigned	*gpioclear;

    // prepare addresses
    gpioset = &GPIO_SET;
    gpioclear = &GPIO_CLR;


    offset1 = DSHOT_T0H_ns - DSHOT_AD_HOC_OFFSET;
    offset2 = DSHOT_T0H_ns;
    offset3 = DSHOT_BIT_ns - offset1 - offset2;
    
    // relax to OS for a small period of time, hope it reduces the probability that we will
    // be interrupted during broadcasting.
    usleep(USLEEP_BEFORE_BROADCAST);	
    // send dshot frame bits
    t = dshotGetNanoseconds();
    for(i=0; i<16; i++) {
	t += offset3;
	while (dshotGetNanoseconds() < t) ;
	*gpioset = allMotorsPinMask;
	t += offset1;
	while (dshotGetNanoseconds() < t) ;
	*gpioclear = clearMasks[i];
	t += offset2;
	while (dshotGetNanoseconds() < t) ;
	*gpioclear = allMotorsPinMask;
    }
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
    int  	mem_fd;
    int32_t	gpioBase;

    gpioBase = getGpioRegBase();
    
    /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("debug Error: Can't open /dev/mem \n");
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


static uint32_t dshotGetAllMotorsPinMask(int motorPins[], int motorMax) {
    int 	i;
    uint32_t 	allMotorsPinsMask;

    // compute masks
    allMotorsPinsMask = 0;
    for(i=0; i<motorMax; i++) allMotorsPinsMask |= (1<<motorPins[i]);
    return(allMotorsPinsMask);
}



//////////////////////////////////////////////////////////////////////////////////////////////
// Main exported functions of the module implementing raspilot motor instance.
//////////////////////////////////////////////////////////////////////////////////////////////

void motorImplementationInitialize(int motorPins[], int motorMax) {
    int i, pin;
    
    dshotSetupIo();
    
    for(i=0; i<motorMax; i++) {
	pin = motorPins[i];
	INP_GPIO(pin); // must use INP_GPIO before we can use OUT_GPIO
	OUT_GPIO(pin);
	GPIO_CLR = 1<<pin;
    }
}

void motorImplementationFinalize(int motorPins[], int motorMax) {
    munmap(dshotGpioMap, BLOCK_SIZE);
}

void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) {
    int		i, bi;
    unsigned	frame[NUM_PINS+1];
    unsigned	bit;
    uint32_t	msk, allMotorsMask;
    uint32_t	clearMasks[16];

    assert(motorMax < NUM_PINS);

    allMotorsMask = dshotGetAllMotorsPinMask(motorPins, motorMax);

    // translate double throttles ranging <0, 1> to dshot frames.
    for(i=0; i<motorMax; i++) frame[i] = dshotAddChecksumAndTelemetry(motorThrottle[i] * 1999 + 48, 0);

    // compute masks for zero bits in all frames
    for(bi=0; bi<16; bi++) {
	msk = 0;
	bit = (0x8000 >> bi);
	for(i=0; i<motorMax; i++) {
	    if ((frame[i] & bit) == 0) msk |= (1<<motorPins[i]);
	}
	clearMasks[bi] = msk;
    }

    dshotSendFrames(allMotorsMask, clearMasks);
}


