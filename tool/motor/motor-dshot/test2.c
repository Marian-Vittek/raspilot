/*
  This example rotates 4 motors, each of them in normal and then reversed direction.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

extern void motorImplementationInitialize(int motorPins[], int motorMax) ;
extern void motorImplementationFinalize(int motorPins[], int motorMax) ;
extern void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) ;
extern void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) ;

#define MAXN 64

int n;
int motorPins[MAXN];
double throttles[MAXN];

int main(int argc, char **argv) {
    int i, k;

    if (argc < 2 || argc >= MAXN) {
	printf("usage:   ./test <gpio_pin_1> ... <gpio_pin_n>\n");
	printf("example: sudo ./test 16 19 20 21\n");
	exit(1);
    }

    n = 0;
    for(i=1; i<argc; i++) {
	motorPins[n] = atoi(argv[i]);
	if (motorPins[n] > 0 && motorPins[n] < 28) {
	    n ++;
	} else {
	    printf("pin %d out of range 1..27\n", motorPins[n]);
	}
    }
    
    motorImplementationInitialize(motorPins, n);
    // motorImplementationSet3dModeAndSpinDirection(motorPins, n, 1, 0);

    // make spinning 1 motor after another
    for(i=0; i<n; i++) throttles[i] = 0;
    for(k=0;k<12;k++) {
	// spin
	throttles[k%4] = 0.1;
	for(i=0; i<100; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(10000);
	}
	// stop
	throttles[k%4] = 0;
	for(i=0; i<100; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(10000);
	}
#if 0	
	// inverse spin
	throttles[k%4] = -0.1;
	for(i=0; i<100; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(10000);
	}
	// stop
	throttles[k%4] = 0;
	for(i=0; i<100; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(10000);
	}
#endif	
    }
    
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);

    // finalize
    motorImplementationFinalize(motorPins, n);
    
    return(0);
}
