#include <stdio.h>
#include <unistd.h>

extern void motorImplementationInitialize(int motorPins[], int motorMax) ;
extern void motorImplementationFinalize(int motorPins[], int motorMax) ;
extern void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) ;


// let's say we have 4 motors connected to GPIO6, GPIO13, GPIO19 and GPIO26
#define N 4
int motorPins[N] = {6, 13, 19, 26};

double throttles[N];

int main() {
    int i, j, k;

    motorImplementationInitialize(motorPins, N);

    // send 0 throttle during 5 seconds
    for(i=0; i<N; i++) throttles[i] = 0;
    for(i=0; i<500; i++) {
        motorImplementationSendThrottles(motorPins, N, throttles);
        usleep(10000);
    }

    // make motors spinning on 15% throttle during 5 seconds
    for(k=0;k<999;k++) {
	for(i=0; i<N; i++) throttles[i] = 0;
	throttles[k%4] = 0.15;
	for(i=0; i<500; i++) {
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
