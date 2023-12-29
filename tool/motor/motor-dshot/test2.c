/*
  This example rotates 4 motors, each of them in normal and then reversed direction.
 */
#include <stdio.h>
#include <unistd.h>

extern void motorImplementationInitialize(int motorPins[], int motorMax) ;
extern void motorImplementationFinalize(int motorPins[], int motorMax) ;
extern void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) ;
extern void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) ;

// let's say we have 4 motors connected to GPIO6, GPIO13, GPIO19 and GPIO26
#define n 4
//int motorPins[N] = {6, 13, 19, 26};
int motorPins[n] = {6, 13, 19, 26};

double throttles[n];

int main() {
    int i, k;

    motorImplementationInitialize(motorPins, n);
    motorImplementationSet3dModeAndSpinDirection(motorPins, n, 1, 0);

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
    }
    
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);

    // finalize
    motorImplementationFinalize(motorPins, n);
    
    return(0);
}
