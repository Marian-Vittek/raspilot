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
#define N 4
//int motorPins[N] = {6, 13, 19, 26};
int motorPins[N] = {6, 13, 19, 26};

double throttles[N];

int main() {
    int i, k;

    motorImplementationInitialize(motorPins, N);
    motorImplementationSet3dModeAndSpinDirection(motorPins, N, 1, 0);

    // make spinning 1 motor after another
    for(i=0; i<N; i++) throttles[i] = 0;
    for(k=0;k<12;k++) {
	// spin
	throttles[k%4] = 0.1;
	for(i=0; i<100; i++) {
	    motorImplementationSendThrottles(motorPins, N, throttles);
	    usleep(10000);
	}
	// stop
	throttles[k%4] = 0;
	for(i=0; i<100; i++) {
	    motorImplementationSendThrottles(motorPins, N, throttles);
	    usleep(10000);
	}
	// inverse spin
	throttles[k%4] = -0.1;
	for(i=0; i<100; i++) {
	    motorImplementationSendThrottles(motorPins, N, throttles);
	    usleep(10000);
	}
	// stop
	throttles[k%4] = 0;
	for(i=0; i<100; i++) {
	    motorImplementationSendThrottles(motorPins, N, throttles);
	    usleep(10000);
	}
    }
    
    // stop motors
    for(i=0; i<N; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, N, throttles);

    // finalize
    motorImplementationFinalize(motorPins, N);
    
    return(0);
}
