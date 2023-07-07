#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "pi2c.h"
#include "Adafruit_PWMServoDriver.h"
#include "../motor-common.h"

// Check your ESC documentation to learn the max frequency at which it can work
#define MOTOR_PWM_FREQUENCY_HZ 		200
#define THROTTLE_TO_ACTIVE_TICKS(t)   	(((t) * 6 + 3) * MOTOR_PWM_FREQUENCY_HZ)

Adafruit_PWMServoDriver pwm;

void motorImplementationInitialize(int motorPins[], int motorMax) {
    pwm.begin();
    /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
    pwm.setOscillatorFrequency(27000000);
    // Analog servos run at ~50 Hz updates
    // However, drone motors ESC usually claim PWM frequency up to 400Hz
    pwm.setPWMFreq(MOTOR_PWM_FREQUENCY_HZ);  
}

void motorImplementationFinalize(int motorPins[], int motorMax) {
      pwm.end();
}

void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) {
    if (mode3dFlag || reverseDirectionFlag) {
	printf("debug Error: This motor does not support 3d mode and/or reverseDirection.\n");
	fflush(stdout);	
    }
}

void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) {
  int i;
  for(i=0; i<motorMax; i++) pwm.setPin(motorPins[i], THROTTLE_TO_ACTIVE_TICKS(motorThrottle[i]), false);
}

