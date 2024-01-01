#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

extern void motorImplementationInitialize(int motorPins[], int motorMax) ;
extern void motorImplementationFinalize(int motorPins[], int motorMax) ;
extern void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) ;


#define MAXN 	40

int motorPins[MAXN];
double throttles[MAXN];

int main(int argc, char **argv) {
    int i, n;

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

    printf("Initializing ESC / Arm, waiting 5 seconds.\n");
    // send 0 throttle during 5 seconds
    for(i=0; i<n; i++) throttles[i] = 0;
    for(i=0; i<5000; i++) {
        motorImplementationSendThrottles(motorPins, n, throttles);
        usleep(1000);
    }

    printf("Spinning.\n");
    // make motors spinning on 15% throttle during 5 seconds
    for(i=0; i<n; i++) throttles[i] = 0.15;
    for(i=0; i<5000; i++) {
	motorImplementationSendThrottles(motorPins, n, throttles);
	usleep(1000);
    }
    
    printf("Stop.\n");
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);

    // finalize
    motorImplementationFinalize(motorPins, n);
    
    return(0);
}
