#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#include "pi2c.h"

void pi2cClose(int fd) {
  close(fd);
}

// path is usualy "/dev/i2c-1"
int pi2cOpen(char *path, int devAddr) {
    int fd;
    fd = open(path,  O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "%s:%d: Can't open %s: %s\n", __FILE__, __LINE__, path, strerror(errno));
	return(-1);
    }
    
    if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        fprintf(stderr, "%s:%d: Can't select device: %s\n", __FILE__, __LINE__, strerror(errno));
        pi2cClose(fd);
        return(-1);
    }
    return(fd);
}

int pi2cReadBytes(int fd, uint8_t regAddr, uint8_t length, uint8_t *data) {
    int r, count;

    r = write(fd, &regAddr, 1);
    if (r != 1) {
        fprintf(stderr, "%s:%d: Failed to write reg: %s\n", __FILE__, __LINE__, strerror(errno));
        return(-1);
    }
    count = read(fd, data, length);
    if (count < 0) {
        fprintf(stderr, "%s:%d: Failed to read device(%d): %s\n", __FILE__, __LINE__, count, strerror(errno));
        return(-1);
    } else if (count != length) {
        fprintf(stderr, "%s:%d: Short read  from device, expected %d, got %d\n", __FILE__, __LINE__, length, count);
        return(-1);
    }
    return count;
}

int pi2cWrite(int fd, uint8_t* data, int length) {
    int count;

    if (length > 127) {
        fprintf(stderr, "%s:%d: Byte write count (%d) > 127\n", __FILE__, __LINE__, length);
        return(-1);
    }

    count = write(fd, data, length);
    if (count < 0) {
        fprintf(stderr, "%s:%d: Failed to write device(%d): %s\n", __FILE__, __LINE__, count, strerror(errno));
        return(-1);
    } else if (count != length) {
        fprintf(stderr, "%s:%d: Short write to device, expected %d, got %d\n", __FILE__, __LINE__, length+1, count);
        return(-1);
    }
    return(count);
}

int pi2cWriteBytesToReg(int fd, uint8_t regAddr, uint8_t length, uint8_t* data) {
    int8_t count = 0;
    uint8_t buf[128];

    buf[0] = regAddr;
    memcpy(buf+1,data,length);
    count = pi2cWrite(fd, buf, length+1);
    return(count);
}

int pi2cWriteWordsToReg(int fd, uint8_t regAddr, uint8_t length, uint16_t* data) {
    int8_t count = 0;
    uint8_t buf[128];
    int i;

    buf[0] = regAddr;
    for (i = 0; i < length; i++) {
        buf[i*2+1] = data[i] >> 8;
        buf[i*2+2] = data[i];
    }
    count = pi2cWrite(fd, buf, length*2+1);
    return(count);
}




