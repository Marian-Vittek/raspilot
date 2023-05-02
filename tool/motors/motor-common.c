#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>

#include "motor-common.h"

#define THROTTLE_MAX		1000
#if 0
#define THROTTLE_LANDING	200
#else
#define THROTTLE_LANDING	0
#endif

int debug = 0;

// TODO: Maybe replace pipes with shared memeory

/*

Motor controller is a separate task teking some argument determining number and connection of each motor.
A usual invocation is 

  motor <pin0> ... <pin_n> 

where n is the number of motors and pin_i is the connection to i-th motor. When started this task is
reading stdin for the lines containign one of the following commands:

  "mt<n> <throttle0> ... <throttle_n>" - set the rotation of n-motors (n must match the number of command line arguments)
                                         throttle_i is in the range 0 - 1000.
  "hb" - heartbeat (keep previous throttle)
  "stan" - goto stanby mode (i.e. send minimal throttle pwn to hold motors stopped
  "fin" or "stop" - stop motors and exit
  "land" - try to land (without gyro it means sending some constant pwm for some time then exit)
  "info" - get status informations (not yet implemented)
  "ping<n>" - answer witn "pong<n>" to stdout (used to determine communication latency)

*/

#if 0
#define MOTOR_READ_BUFFER_SIZE 			1024
#define MOTOR_EMERGENCY_LANDING_TIME_SECONDS 	5
#else
#define MOTOR_READ_BUFFER_SIZE 			1024
#define MOTOR_EMERGENCY_LANDING_TIME_SECONDS 	1
#endif

int motorShutdownInProgress = 0;
int motorEmergencyLandingInProgress = 0;
int motorStandBy = 1;

int motorPins[MOTOR_MAX];
double motorThrottle[MOTOR_MAX];
int motorMax = 0;

int inputPipeFd;
int outputPipeFd;

// TODO, this shall probably be specified on a per motor bases
double motorThrottleSafeLand[MOTOR_MAX];


#define SKIP_BLANK(p) { while (*p != 0 && isspace(*p)) p++; }

static uint64_t currentTimestampUsec() {
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    return((uint64_t)tv.tv_sec * 1000000LL + tv.tv_usec);
}

char *sprintTime_st(long long int utime) {
    time_t          t;
    int             u;
    struct tm       *tm, ttm;
    static char	    res[256];
    
    t = utime / 1000000;
    u = utime % 1000000;
    tm =  localtime_r(&t, &ttm);
    snprintf(res, sizeof(res)-1, "%4d-%02d-%02d %02d:%02d:%02d.%03d", 
	     1900+tm->tm_year, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec,
	     u/1000);
    return(res);
}

void motorSendThrottles() {
    int 	i;
    uint64_t 	t;

    if (debug) {
	t = currentTimestampUsec();
	printf("debug %s: motorSendThrottles: ", sprintTime_st(t));
	for(i=0; i<motorMax; i++) printf("%f ", motorThrottle[i]);
	printf("\n");
	fflush(stdout);
    }
    motorImplementationSendThrottles(motorPins, motorMax, motorThrottle);
}

void motorStartEmergencyLanding() {
    int i;
    // This is some emergency stop. If there is an error on the input, try to land with minimal damage.
    motorEmergencyLandingInProgress = 1;
    for(i=0; i<motorMax; i++) motorThrottle[i] = motorThrottleSafeLand[i];
    motorSendThrottles();
    usleep(100000);
}

int readThrustFromInputPipe() {
    static char		pongbuf[255];
    static char		bbb[MOTOR_READ_BUFFER_SIZE];
    static int		bbbi = 0;

    int 		i, n, t;
    long long		tstamp;
    char		*p, *q, *eq;
    int 		r, d, res;
    fd_set 		inset;
    fd_set 		outset;
    fd_set 		errset;
    struct timeval 	tout;

    res = 0;
    if (motorShutdownInProgress) return(0);
    if (motorEmergencyLandingInProgress) return(0);

    // TODO: Maybe do this more efficiently preparing and holding static empty fdsets
    FD_ZERO(&inset);
    FD_SET(inputPipeFd, &inset);
    FD_ZERO(&outset);
    FD_ZERO(&errset);
    FD_SET(inputPipeFd, &errset);
    tout.tv_sec = 0;
    
    if (debug > 10) {
	tout.tv_usec = 200000000;	// wait 200ms max
    } else {
	tout.tv_usec = 200000;		// wait 200ms max
    }
    
    // printf("["); fflush(stdout);
    r = select(inputPipeFd + 1, &inset, &outset, &errset, &tout);
    // printf("]"); fflush(stdout);

    // on error we shall probably do safe landing, however it may happen that select returns
    // -1 even if select was interrupted
    if (r < 0) {
	// There we should probably check if timeout occured during multiple interruptions ...
	// Anyway for now we do nothing if select was just interrupted
	if (errno == EINTR) return(0);
	// this probably means some more serious error, do emergency landing
	printf("debug Error: %s:%d: select returned %d, errno == %d, emergency landing!\n", __FILE__, __LINE__, r, errno);
	goto emergencyLanding;
    }
    // check for timeout expired
    if (r == 0) {
	if (motorStandBy) return(0);
	printf("debug Error: %s:%d: select returned 0 == timeout, errno == %d, emergency landing!\n", __FILE__, __LINE__, errno);
	printf("debug Current timestamp == %s\n", sprintTime_st(currentTimestampUsec()));
	fflush(stdout);
	goto emergencyLanding;
    }
    // if there is some problem with stdin, do emergency landing
    if (FD_ISSET(inputPipeFd, &errset)) {
	printf("debug Error: %s:%d: select reported exception on stdin, emergency landing!\n", __FILE__, __LINE__);	
	goto emergencyLanding;
    }
    
    // Hmm. what to do if nothing is signaled, no timeout, no error and still there is
    // nothing on stdin?
    if (! FD_ISSET(inputPipeFd, &inset)) {
	printf("debug Error: %s:%d: select reported no input on stdin, emergency landing!\n", __FILE__, __LINE__);	
	goto emergencyLanding;
    }
    if (bbbi >= MOTOR_READ_BUFFER_SIZE - 2) {
	printf("debug Error: %s:%d: read buffer full before read, emergency landing!\n", __FILE__, __LINE__);	
	goto emergencyLanding;
    }
    // o.k. we have something on stdin
    n = read(inputPipeFd, bbb+bbbi, MOTOR_READ_BUFFER_SIZE - bbbi - 2);
    if (n == 0) {
	// surprising this seems to happens when the writing end of pipe is closed
	usleep(10000);
	if (motorStandBy) return(0);
	printf("debug Error: %s:%d: read has read nothing!\n", __FILE__, __LINE__);
	// return(0);
	goto emergencyLanding;
    }
    bbbi += n;
    if (bbbi >= MOTOR_READ_BUFFER_SIZE) {
	printf("debug Error: %s:%d: read buffer overflowed! Emergency landing!\n", __FILE__, __LINE__);		
	goto emergencyLanding;
    }
    bbb[bbbi] = 0;
    for (;;) {
	p = strchr(bbb, '\n');
	// no more newlines read we are done with parsing
	if (p == NULL) return(res);
    	*p = 0;
	// o.k. we have whole line, parse it
	q = bbb;
	if (debug) {
	    uint64_t 	t;
	    t = currentTimestampUsec();
	    printf("debug %s: %s:%d: Parsing '%s'\n", sprintTime_st(t), __FILE__, __LINE__, bbb);
	}
	SKIP_BLANK(q);
	if (q[0] == 'h' && q[1] == 'b' && q[2] == 0) {
	    // heartbeat, meaning hold last pwm
	    res |= 0;
	} else if (q[0] == 'm' && q[1] == 't') {
	    q += 2;
	    n = strtol(q, &eq, 10);
	    if (eq == q) goto emergencyLanding;
	    if (n != motorMax) goto emergencyLanding;
	    q = eq;
	    for(i=0; i<n; i++) {
		t = strtol(q, &eq, 10);
		if (eq == q) goto emergencyLanding;
		q = eq;
		if (t < 0 || t > THROTTLE_MAX) goto emergencyLanding;
		motorThrottle[i] = ((double)t)/THROTTLE_MAX;
	    }
	    // since that moment watch for timeouts until new standby mode activated
	    motorStandBy = 0;
	    res |= 1;
	} else if (q[0] == 's' && q[1] == 't' && q[2] == 'a' && q[3] == 'n') {
	    // stand bye. This happens when raspilot exits, wait until a new connection happens
	    motorStandBy = 1;
	    res |= 1;
	} else if ((q[0] == 'f' && q[1] == 'i' && q[2] == 'n')
		   || (q[0] == 's' && q[1] == 't' && q[2] == 'o' && q[2] == 'p')) {
	    motorShutdownInProgress = 1;
	    for(i=0; i<motorMax; i++) motorThrottle[i] =  0;
	    res |= 1;
	} else if (q[0] == 'l' && q[1] == 'a' && q[2] == 'n' && q[3] == 'd') {
	    // raspilot asked for emergency landing
	    goto emergencyLanding;
	} else if (q[0] == 'p' && q[1] == 'i' && q[2] == 'n' && q[3] == 'g') {
	    tstamp = atoll(q+4);
	    r = snprintf(pongbuf, sizeof(pongbuf)-1, "pong%lld\n", tstamp);
	    write(outputPipeFd, pongbuf, r);
	    res |= 0;
	} else if (q[0] == 'i' && q[1] == 'n' && q[2] == 'f' && q[3] == 'o') {
	    // raspilot status information, ignore
	    res |= 0;
	} else if (q[0] == 0) {
	    // empty line, ignore
	    res |= 0;
	} else {
	    // some wrong input, do emergency landing
	    printf("debug Error: %s:%d: wrong input line: %s! Emergency landing!\n", __FILE__, __LINE__, bbb);	
	    goto emergencyLanding;
	}

	// TODO: do this without physically moving the rest of the buffer
	d = bbb+bbbi-p-1;
	memmove(bbb, p+1, d);
	bbbi = d;
    }
    
emergencyLanding:
    motorStartEmergencyLanding();
    return(1);
}

void motorStopInterrupt(int signum) {
    printf("debug %s:%d: Interrupt %d received. Shutdown.\n", __FILE__, __LINE__, signum);
    motorShutdownInProgress = 1;
}

void motorIgnoreInterrupt(int signum) {
    printf("debug %s:%d: Ingored interrupt %d received.\n", __FILE__, __LINE__, signum);
}

void motorFatalInterrupt(int signum) {
    printf("debug %s:%d: Fatal interrupt %d, core dump.\n", __FILE__, __LINE__, signum);
    motorImplementationFinalize(motorPins, motorMax);
    signal(SIGABRT, SIG_DFL);
    abort();
}

// TODO: Use this to synchronize raspilot ticks to motor PWM ticks
void pinalert(int gpio, int level, uint32_t tick) {
    printf("debug get %d pin level %d\n", gpio, level);
}

int main(int argc, char *argv[]) {
    int i, r, g;
    
    if (argc < 3) {
	printf("debug Usage: motor <input_pipe_path> <output_pipe_path> <gpio_0> ... <gpion>\n");
	exit(-1);
    }

    signal(SIGINT, motorStopInterrupt);
    
    if (strcmp(argv[1], "-") == 0) {
	inputPipeFd = 0;
    } else {
	inputPipeFd = open(argv[1], O_RDWR);
    }
    if (inputPipeFd < 0) {
	printf("debug %s:%d: Can't open input pipe;\n", __FILE__, __LINE__);
	exit(-1);
    }

    if (strcmp(argv[2], "-") == 0) {
	outputPipeFd = 1;
    } else {
	outputPipeFd = open(argv[2], O_RDWR);
    }
    if (outputPipeFd < 0) {
	printf("debug %s:%d: Can't open output pipe;\n", __FILE__, __LINE__);
	exit(-1);
    }
    
    for (i=0; i<argc-3; i++) {
	g = atoi(argv[i+3]);
	motorPins[i] = g;
	motorThrottle[i] = 0;
	// motorThrottleSafeLand[i] = 1500;
	motorThrottleSafeLand[i] = ((double)THROTTLE_LANDING) / THROTTLE_MAX;
    }
    motorMax = i;
   
    motorImplementationInitialize(motorPins, motorMax);

    motorSendThrottles();
    // This is the main loop reading and executing PWMs
    while (! motorShutdownInProgress && !motorEmergencyLandingInProgress) {
	r = readThrustFromInputPipe();
	if (r) motorSendThrottles();
	// usleep(1000);
    }

    if (motorEmergencyLandingInProgress) sleep(MOTOR_EMERGENCY_LANDING_TIME_SECONDS);
    
    // send min_width to motors to stop them before exit
    for(i=0; i<motorMax; i++) motorThrottle[i] = 0;
    motorSendThrottles();
    
    usleep(100000);

    motorImplementationFinalize(motorPins, motorMax);
    return 0;
}
