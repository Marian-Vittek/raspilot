#include <stdio.h>
#include <unistd.h>

extern void motorImplementationInitialize(int motorPins[], int motorMax) ;
extern void motorImplementationFinalize(int motorPins[], int motorMax) ;
extern void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) ;


// let's say we have 4 motors connected to GPIO12, GPIO16, GPIO20 and GPIO21
#define N 4
int motorPins[N] = {12, 16, 20, 21};

double throttles[N];

int main() {
    int i;

    motorImplementationInitialize(motorPins, N);

    printf("Initializing ESC / Arm, waiting 5 seconds.\n");
    fflush(stdout);
    // send 0 throttle during 5 seconds
    for(i=0; i<N; i++) throttles[i] = 0;
    for(i=0; i<5000; i++) {
        motorImplementationSendThrottles(motorPins, N, throttles);
        usleep(1000);
    }

    printf("Spinning.\n");
    fflush(stdout);
    // make motors spinning on 15% throttle during 5 seconds
    for(i=0; i<N; i++) throttles[i] = 0.15;
    for(i=0; i<5000; i++) {
	motorImplementationSendThrottles(motorPins, N, throttles);
	usleep(1000);
    }
    
    printf("Stop.\n");
    fflush(stdout);
    // stop motors
    for(i=0; i<N; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, N, throttles);

    // finalize
    motorImplementationFinalize(motorPins, N);
    
    return(0);
}
