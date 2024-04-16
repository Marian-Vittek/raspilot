//
// This tasks allows access to Raspberry Pi hardware PWM on GPIO pins 12 and 13.
//
// It is reading standard input expecting lines of the format
// cX VVVV
// and sending value proportional to VVVV to servo on hardware PWM channel X.
//
// For example:
//   echo c0 5000 | sudo ./hw-pwm
// shall turn servo on GPIO12 to middle position.
//   echo c0 0 | sudo ./hw-pwm
// and
//   echo c0 9999 | sudo ./hw-pwm
// shall turn servo on GPIO12 to extremities positions.
//
// based on pwm.c example from bcm2835 library
//

#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// Specify how fine is the movement (granularity of the output). Smaller divider makes the movement smoother.
// Seems that the value 2 is the smallest working
#define PWM_USE_CLOCK_DIVIDER 	BCM2835_PWM_CLOCK_DIVIDER_2

// Specify how many updates (pulses) are sent per second. Analog servos may require 20Hz. Digital should work up to 400Hz. 
static int hwPwmFrequency=20;
// The minimal pulse length (moving servo to min position)
static int hwPwmMinPulseUs=600;
// The maximal pulse length (moving servo to max position)
static int hwPwmMaxPulseUs=2000;
// The input coming from stdin is between 0 - hwPwmInputFactor, generating pulses of lengths between hwPwmMinPulseUs - hwPwmMaxPulseUs.
static int hwPwmInputFactor=10000;


// PWM output on RPi GPIO pins 12 and 13 (hardware pwm)
#define PIN0 12
#define PIN1 13

// controlled by PWM channels 0
#define PWM_CHANNEL0 0
#define PWM_CHANNEL1 1

//
#define PWM_INPUT_OUT_OF_RANGE(value) ((value) < 0 || (value) > hwPwmInputFactor)

// This control when the next peak is broadcasted.
static int hwPwmRange=24000;

void printUsageAndExit() {
    printf("usage: hw-pwm <min_pulse_length_us> <max_pulse_length_us> <pwm_frequency> <c0_VVVV_initial_value> <c1_VVVV_initial_value>\n");
    exit(0);
}

int hwPwmInputToData(int value) {
    int res;
    // res = (value * 3 + 10000) * 3 / 50;
    // res = value / factor * (hwPwmMaxPulseUs - hwPwmMinPulseUs) + hwPwmMinPulseUs;
    res = (value * (hwPwmMaxPulseUs - hwPwmMinPulseUs) / hwPwmInputFactor + hwPwmMinPulseUs) * 192 / (PWM_USE_CLOCK_DIVIDER * 10);
    return(res);
}

int main(int argc, char **argv) {
    char	*line, *p, *q, *r, *s, *t;
    size_t	lineSize;
    int		channel;
    int		i, v0, v1, value;
    int 	data;

    if (argc != 6) printUsageAndExit();

    i = 1;
    hwPwmMinPulseUs = strtol(argv[i++], &p, 10);
    hwPwmMaxPulseUs = strtol(argv[i++], &q, 10);
    hwPwmFrequency = strtol(argv[i++], &r, 10);
    v0 = strtol(argv[i++], &s, 10);
    v1 = strtol(argv[i++], &t, 10);

    if (argv[1] == p || argv[2] == q || argv[3] == r || argv[4] == s || argv[5] == t) printUsageAndExit();

    if (hwPwmFrequency < 1 || hwPwmFrequency > 400) {
	printf("debug: PWM frequency %d out of of range.\n", hwPwmFrequency);
	exit(-1);
    }
    
    if (PWM_INPUT_OUT_OF_RANGE(v0) || PWM_INPUT_OUT_OF_RANGE(v1)) {
	printf("debug: PWM Initial values %d,%d of range.\n", v0, v1);
	exit(-1);
    }
    
    if (! bcm2835_init()) return 1;

    hwPwmRange= (19200000 / PWM_USE_CLOCK_DIVIDER / hwPwmFrequency);
    
    // Set the output pin to Alt Fun 0, to allow PWM channel there
    bcm2835_gpio_fsel(PIN0, BCM2835_GPIO_FSEL_ALT0);
    bcm2835_gpio_fsel(PIN1, BCM2835_GPIO_FSEL_ALT0);

    // Clock divider is set to 8. Range to 60000 the pulse will be
    // 1.2MHz/12000 = 100Hz
    bcm2835_pwm_set_clock(PWM_USE_CLOCK_DIVIDER);
    bcm2835_pwm_set_mode(PWM_CHANNEL0, 1, 1);
    bcm2835_pwm_set_mode(PWM_CHANNEL1, 1, 1);
    bcm2835_pwm_set_range(PWM_CHANNEL0, hwPwmRange);
    bcm2835_pwm_set_range(PWM_CHANNEL1, hwPwmRange);
    usleep(10000);

    // set initial pwm
    bcm2835_pwm_set_data(PWM_CHANNEL0, hwPwmInputToData(v0));
    bcm2835_pwm_set_data(PWM_CHANNEL1, hwPwmInputToData(v1));

    printf("debug: %s: Starting hw-pwm.\n", __FILE__); fflush(stdout);
    printf("debug: %s: MinPulseLength: %d us.\n", __FILE__, hwPwmMinPulseUs); fflush(stdout);
    printf("debug: %s: MaxPulseLength: %d us.\n", __FILE__, hwPwmMaxPulseUs); fflush(stdout);
    printf("debug: %s: InputFactor: %d.\n", __FILE__, hwPwmInputFactor); fflush(stdout);
    printf("debug: %s: Initial values received %d,%d.\n", __FILE__, v0, v1); fflush(stdout);
    
    line = NULL; lineSize = 0;
    while  (getline(&line, &lineSize, stdin) >= 0) {
	// in our simple protocol line starting with 'e' means exit
	if (line[0] == 'e' || line[0] == 'q') {
	    printf("debug: %s: Exiting\n", __FILE__);
	    break;
	} else if (line[0] == 'c') {
	    // line starting with 'c' sets channel and value in the format "c0 VVVV" or "c1 VVVV".
	    // VVVV is in the range 0 - hwPwmInputFactor.
	    p = line+1;
	    channel = strtol(p, &q, 10);
	    if (p == q) {
		printf("debug: %s: no channel number in: %s\n", __FILE__, line); fflush(stdout);
		continue;
	    }
	    if (channel != PWM_CHANNEL0 && channel != PWM_CHANNEL1) {
		printf("debug: %s: wrong channel, expected value %d or %d in: %s\n", __FILE__, PWM_CHANNEL0, PWM_CHANNEL1, line); fflush(stdout);
		continue;
	    }
	    p = q;
	    value = strtol(p, &q, 10);
	    if (p == q) {
		printf("debug: %s: no value in line %s\n", __FILE__, line); fflush(stdout);
		continue;
	    }
	    if (PWM_INPUT_OUT_OF_RANGE(value)) {
		printf("debug: %s: wrong value, expected range 0-%d in: %s\n", __FILE__, hwPwmInputFactor, line); fflush(stdout);
		continue;
	    }
	    p = q;
	    // translate value ranging between 0 - hwPwmInputFactor to PWM .
	    data =  hwPwmInputToData(value);
	    // printf("debug: %s: setting channel %d to pulse %d us, data %d\n", __FILE__, channel, data*10/12, data); fflush(stdout);
	    bcm2835_pwm_set_data(channel, data);
	} else {
	    printf("debug: %s: line does not start with 'c', ignoring line: %s\n", __FILE__, line); fflush(stdout);
	    continue;
	}
    }

    bcm2835_close();
    return 0;
}
