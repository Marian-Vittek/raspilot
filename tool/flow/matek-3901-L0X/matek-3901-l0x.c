/*

  This is a simple program reading MSP V2 messages from 
  Mateksys optical flow & lidar sensor 3901-L0X and printing
  range and motion on standard output.

 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

#include "msp_protocol.h"

int baudrateToSpeed_t(int baudrate) {
    switch (baudrate) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default: 
        return 0;
    }
}

void initSerialPort(int fd, int baudrate) {
    struct termios 	tty;
    speed_t			speed;
	
    memset (&tty, 0, sizeof tty);

    if ( tcgetattr ( fd, &tty ) != 0 ) {
	printf("%s:%d: Error: tcgetattr: %s\n", __FILE__, __LINE__, strerror(errno));
    }

    speed = baudrateToSpeed_t(baudrate);
    if (speed <= 0) {
	printf("%s:%d: Error: Invalid serial port baud rate %d. Using 9600 instead!\n", __FILE__, __LINE__, baudrate);
	speed = B9600;
    }

#if 0
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cflag     &=  ~ECHO;              // no echo
    tty.c_cflag     &=  ~ICRNL;
    tty.c_cflag     &=  ~IXON;
    tty.c_cflag     &=  ~IXOFF;
    tty.c_cflag     &=  ~OPOST;
    tty.c_cflag     &=  ~ISIG;
    tty.c_cflag     &=  ~ICANON;

    tty.c_cc[VMIN]   =  1;
    tty.c_cc[VTIME]  =  5;
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
#endif
	
    cfmakeraw(&tty);

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    // This flushes all the input in the buffer but not read yet, so we start with newly read stuff
    tcflush( fd, TCIFLUSH );

    if ( tcsetattr ( fd, TCSANOW, &tty ) != 0) {
	printf("%s:%d: Error: tcsetattr: %s\n", __FILE__, __LINE__, strerror(errno));
    }

}

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
    int ii;
    crc ^= a;
    for (ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

uint8_t getccrc(FILE *ff, uint8_t *checksum) {
    int c;
    
    c = getc(ff);
    if (c == EOF) {
	printf("%s:%d: Error: Unexpected end of input stream.\n", __FILE__, __LINE__);
	exit(-1);
    }
    *checksum = crc8_dvb_s2(*checksum, (unsigned char)c);
    return(c);
}


int mspSkipUntilMsgStart(FILE *ff) {
    int c;
    
    while ((c=getc(ff)) != '$' && c != EOF) ;
    if (c == EOF) {
	printf("%s:%d: Error: Unexpected end of input stream.\n", __FILE__, __LINE__);
	exit(-1);
    }
    ungetc(c, ff);
}

// last read altitude
double lastRange = 0;

// We report motion in degrees of the angle by which viewport shifted (rotated).
// matek has 35x35 pixels in 42 degree range, No idea from where the constant 10 comes
// I guessed it by experimentations. Maybe matek is reporting in pixel*10?
// factor == 42 / 180 * M_PI / 35.0 / 10.0 
#define FACTOR 0.002094395102393196

static inline double motionToAlpha(double p) {
    return(p * FACTOR);
}

int mspScanMessage(FILE *ff) {
    int 	i, c;
    int 	function, payloadSize;
    int 	quality;
    int32_t 	rangeMm, motionX, motionY;
    int 	ck;
    uint8_t 	checksum;

    // Scan message prefix
    c = getc(ff);
    if (c != '$') {
	printf("%s:%d: Error: Message does not start with '$'.\n", __FILE__, __LINE__);
	return(-1);
    }
    c = getc(ff);
    if (c != 'X') {
	printf("%s:%d: Error: Expected 'X'.\n", __FILE__, __LINE__);
	ungetc(c, ff);
	return(-1);
    }
    c = getc(ff);
    if (c != '<') {
	printf("%s:%d: Expected '<'.\n", __FILE__, __LINE__);
	ungetc(c, ff);
	return(-1);
    }

    // Scan rest of data and compute checksum
    checksum = 0;
    c = getccrc(ff, &checksum);
    function = (int16_t)(getccrc(ff, &checksum) + (getccrc(ff, &checksum) << 8)) ;
    payloadSize = (int16_t)(getccrc(ff, &checksum) + (getccrc(ff, &checksum) << 8));
    switch (function) {
    case MSP2_SENSOR_RANGEFINDER:
	if (payloadSize != 5) {
	    printf("%s:%d: MSP2_SENSOR_RANGEFINDER unexpected payloadSize %d.\n", __FILE__, __LINE__, payloadSize);
	    return(-1);
	}
	quality = getccrc(ff, &checksum);
	rangeMm = (int32_t)(getccrc(ff, &checksum) + (getccrc(ff, &checksum) << 8)  + (getccrc(ff, &checksum) << 16) + (getccrc(ff, &checksum) << 24));
	ck = getc(ff);
	if (ck != checksum) {
	    printf("%s:%d: Error: wrong checksum.\n", __FILE__, __LINE__);
	    return(-1);
	}
	// When in motion, very often it reports range -1 with quality == 255;
	// Avoid such cases and do not report anything when out of range
	if (rangeMm >= 20 && rangeMm <= 2000) {
	    printf("range %5.3f %4.2f\n", rangeMm/1000.0, quality/255.0);
	    // printf("Got MSP2_SENSOR_RANGEFINDER: quality: %3d; distanceMm: %5d\n", quality, distanceMm);
	    lastRange = rangeMm/1000.0;
	}
	break;
    case MSP2_SENSOR_OPTIC_FLOW:
	if (payloadSize != 9) {
	    printf("%s:%d: MSP2_SENSOR_OPTIC_FLOW unexpected payloadSize %d.\n", __FILE__, __LINE__, payloadSize);
	    return(-1);
	}
	quality = getccrc(ff, &checksum);
	motionX = (int32_t)(getccrc(ff, &checksum) + (getccrc(ff, &checksum) << 8)  + (getccrc(ff, &checksum) << 16) + (getccrc(ff, &checksum) << 24));
	motionY = (int32_t)(getccrc(ff, &checksum) + (getccrc(ff, &checksum) << 8)  + (getccrc(ff, &checksum) << 16) + (getccrc(ff, &checksum) << 24));
	ck = getc(ff);
	if (ck != checksum) {
	    printf("%s:%d: Error: wrong checksum.\n", __FILE__, __LINE__);
	    return(-1);
	}
	// printf("Got MSP2_SENSOR_OPTIC_FLOW: quality: %3d; motionX: %5d, motionY: %5d\n", quality, motionX, motionY);
	// printf("pixelmotion %5.3f %5.3f %4.2f\n", -motionY/1000.0, -motionX/1000.0, quality/255.0);
	
	// Actually print the motion in drone frame (not the sensor frame) and translated to angles (not sensor pixels).
	printf("motion %5.3f %5.3f %4.2f\n", motionToAlpha(-motionY), motionToAlpha(-motionX), quality/255.0);
	break;
    default:	
	printf("%s:%d: Unexpected message type %04x of size %d.\n", __FILE__, __LINE__, function, payloadSize);
	return(-1);
    }
    return(0);

}

int main(int argc, char **argv) {
    char	*fname;
    FILE 	*ff;
    int 	r;

    if (argc < 2) {
	fname = "/dev/serial0";
    } else {
	fname = argv[1];
    }
    
    ff = fopen(fname, "rw");
    if (ff == NULL) {
	printf("%s:%d: Can't open serial port %s.\n", __FILE__, __LINE__, fname);
	exit(-1);
    }

    initSerialPort(fileno(ff), 115200);

    for(;;) {
	r = mspScanMessage(ff);
	if (r != 0) mspSkipUntilMsgStart(ff);
	fflush(stdout);
    }
    
    
    fclose(ff);
}
