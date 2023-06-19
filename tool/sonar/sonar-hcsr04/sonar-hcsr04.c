/*
  Meassure the distance using HC-SR04 sensor and Raspberry Pi.
  This program uses pigpio library.
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>
#include <signal.h>

void sonarReceiveAlertFunction(int gpio, int level, uint32_t tick) {
    static uint32_t     setTick;
    
    uint32_t    diff;
    double      dist;
    
    // printf("GPIO %2d became %d at %d\n", gpio, level, tick);
    if (level == 1) {
        setTick = tick;
    } else if (level == 0 && setTick != 0) {
        diff =  tick - setTick;
        // diff is in microseconds, speed of sound is 343 m/s and the
        // distance is half of the signal trip
        dist = diff * 343.0 / 1000000.0 / 2;
        if (dist < 0) dist = 0;
        if (dist > 50) dist = -1;
        printf("dist %6.4f\n", dist); fflush(stdout);
        setTick = 0;
    }
}

void taskStop(int signum) {
    gpioTerminate();
    exit(0);
}

int main(int argc, char **argv) {
    int pinSend;
    int pinReceive;

    if (argc < 3) {
        printf("Usage: sonar-hcsr04 sending_pin receiving_pin\n");
        exit(-1);
    }
    pinSend = atoi(argv[1]);
    pinReceive = atoi(argv[2]);

    if (gpioInitialise() < 0) {
        printf("debug Error: Initialisation of the GPIO Failed.\n");
        exit(-1);
    }

    signal(SIGINT, taskStop);
  
    gpioSetAlertFunc(pinReceive, sonarReceiveAlertFunction);
  
    for(;;) {
        // Do samples at (around) 100Hz 
        usleep(9800);
        gpioWrite(pinSend, 1);
        usleep(10);
        gpioWrite(pinSend, 0);
    }
    
    taskStop(0);
}
