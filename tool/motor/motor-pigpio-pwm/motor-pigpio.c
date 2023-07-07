#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <pigpio.h>
#include "../motor-common.h"


#define MOTOR_PWM_FREQUENCY_HZ 		 200

#define MOTOR_PWM_MIN_WIDTH             1100
#define MOTOR_PWM_MAX_WIDTH             1900

#define THROTTLE_TO_PWM(t) 		((t) * (MOTOR_PWM_MAX_WIDTH - MOTOR_PWM_MIN_WIDTH) + MOTOR_PWM_MIN_WIDTH)


static void motorIgnoreInterrupt(int signum) {
    printf("debug %s:%d: Ingored interrupt %d received.\n", __FILE__, __LINE__, signum);
}

static void motorFatalInterrupt(int signum) {
    printf("debug %s:%d: Fatal interrupt %d, core dump.\n", __FILE__, __LINE__, signum);
    gpioTerminate();
    signal(SIGABRT, SIG_DFL);
    abort();
}

void motorImplementationInitialize(int motorPins[], int motorMax) {
    int i;
    
    if (gpioInitialise() < 0) {
	printf("debug Error: Can't initialize pigpio\n");
	fflush(stdout);
    }

    // this seems to exhaust some pigpio resource
    gpioSetSignalFunc(SIGSEGV, motorFatalInterrupt);
    gpioSetSignalFunc(SIGPIPE, motorIgnoreInterrupt);

    // PWM Hz    50   100  200  400  500
    // 1E6/Hz 20000 10000 5000 2500 2000
    
    for(i=0; i<motorMax; i++) {
	gpioSetPWMfrequency(motorPins[i], MOTOR_PWM_FREQUENCY_HZ);
	gpioSetPWMrange(motorPins[i], 1000000/MOTOR_PWM_FREQUENCY_HZ);
    }
}

void motorImplementationFinalize(int motorPins[], int motorMax) {
    gpioTerminate();
}

void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) {
    if (mode3dFlag || reverseDirectionFlag) {
	printf("debug Error: This motor does not support 3d mode and/or reverseDirection.\n");
	fflush(stdout);	
    }
}

void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) {
  int i;
  for(i=0; i<motorMax; i++) {
      // printf("debug sending %g --> %d to pin %d\n", motorThrottle[i], (int)(THROTTLE_TO_PWM(motorThrottle[i])), motorPins[i]); fflush(stdout);
      gpioPWM(motorPins[i], (int)(THROTTLE_TO_PWM(motorThrottle[i])));
  }
}
