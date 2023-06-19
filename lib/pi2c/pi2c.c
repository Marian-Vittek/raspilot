#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <pthread.h>
#include <semaphore.h>

#include "pi2c.h"

#define I2C_MULTIPROCESS_SHARING 1

////////////////////////////////////////////////////////////////////////////
// multiprocess sync stuff

#define MAX_OPEN_DEV 256

struct fdStr {
    int 	fd;
    sem_t	*sem;
};


static int		pi2cInitializedFlag = -1;
static struct fdStr 	fdTab[MAX_OPEN_DEV];
static int 		fdTabIndex = 0;
static pthread_mutex_t	fdTabMutex = PTHREAD_MUTEX_INITIALIZER;

static void pi2cGetSemaphoreName(char *path, char semName[PATH_MAX]) {
    char	*p;

    // get semaphore name
    snprintf(semName, PATH_MAX, "/%s", path);
    semName[PATH_MAX-1] = 0;
    for(p=semName+1; *p; p++) {
	if (*p == '/') *p = '-';
    }
}

void pi2cInit(char *path, int multiProcessSharingFlag) {
    char 	semName[PATH_MAX];

    if (pi2cInitializedFlag == -1) {
	// printf("pi2cInit(%s, %d)\n", path, multiProcessSharingFlag);
	if (I2C_MULTIPROCESS_SHARING && multiProcessSharingFlag == 0) {
	    pi2cGetSemaphoreName(path, semName);
	    sem_unlink(semName);
	}
	fdTabIndex = 0;
	pi2cInitializedFlag = multiProcessSharingFlag;
    } else {
	if (pi2cInitializedFlag != multiProcessSharingFlag) {
	    fprintf(stderr, "%s:%d: pi2cInit called with different multiProcessSharingFlags\n", __FILE__, __LINE__);
	}
    }
}


// path is usualy "/dev/i2c-1"
int pi2cOpen(char *path, int devAddr) {
    int 	fd, ifd;
    char 	semName[PATH_MAX];

    if (pi2cInitializedFlag == -1) pi2cInit(path, 0);
	
    pthread_mutex_lock(&fdTabMutex);
    ifd = -1;
    
    // printf("pi2cOpen(%s, %02x)\n", path, devAddr); fflush(stdout);
    
    fd = open(path,  O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "%s:%d: Can't open %s: %s\n", __FILE__, __LINE__, path, strerror(errno));
	goto exitPoint;
    }
    
    if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        fprintf(stderr, "%s:%d: Can't select device: %s\n", __FILE__, __LINE__, strerror(errno));
	close(fd);
	goto exitPoint;
    }

    if (I2C_MULTIPROCESS_SHARING) {
	pi2cGetSemaphoreName(path, semName);
	for(ifd=0; ifd<fdTabIndex && fdTab[ifd].fd != -1; ifd ++) ;
	if (ifd >= MAX_OPEN_DEV) {
	    fprintf(stderr, "%s:%d: Too many i2c devices opened. When opening %s %d\n", __FILE__, __LINE__, path, devAddr);
	    close(fd);
	    goto exitPoint;
	}
	fdTab[ifd].sem = sem_open(semName, O_RDWR|O_CREAT, 0666, 1);
	if (fdTab[ifd].sem == SEM_FAILED) {
	    fprintf(stderr, "%s:%d: Can't open semaphore %s: %s\n", __FILE__, __LINE__, semName, strerror(errno));
	    close(fd);
	    goto exitPoint;
	}
	fdTab[ifd].fd = fd;
	if (ifd == fdTabIndex) fdTabIndex ++;
    } else {
	ifd = fd;
    }
    
exitPoint:
    pthread_mutex_unlock(&fdTabMutex);
    return(ifd);
}

void pi2cClose(int ifd) {
    int 	fd;
    
    // printf("pi2cClose(%d)\n", ifd); fflush(stdout);

    if (I2C_MULTIPROCESS_SHARING) {
	sem_close(fdTab[ifd].sem);
	fd = fdTab[ifd].fd;
	fdTab[ifd].fd = -1;
    } else {
	fd = ifd;
    }
    close(fd);
}


int pi2cReadBytes(int ifd, uint8_t regAddr, uint8_t length, uint8_t *data) {
    int r, count;
    int fd;

    // printf("pi2cReadBytes(%d, %d, %d, ...)\n", ifd, regAddr, length); fflush(stdout);
    count = -1;

    if (I2C_MULTIPROCESS_SHARING) {
	sem_wait(fdTab[ifd].sem);
	fd = fdTab[ifd].fd;
    } else {
	fd = ifd;
    }

    r = write(fd, &regAddr, 1);
    if (r != 1) {
        fprintf(stderr, "%s:%d: Failed to write reg: %s\n", __FILE__, __LINE__, strerror(errno));
abort();
        goto exitPoint;
    }
    count = 0;
    while (count < length) {
	r = read(fd, data+count, length-count);
	if (r <= 0) {
	    fprintf(stderr, "%s:%d: Failed to read device(r==%d): %s\n", __FILE__, __LINE__, r, strerror(errno));
            goto exitPoint;
	}
	count += r;
    }

exitPoint:

    if (I2C_MULTIPROCESS_SHARING) {
	sem_post(fdTab[ifd].sem);
    }
    return count;
}

int pi2cWrite(int ifd, uint8_t* data, int length) {
    int r, count;
    int fd;
    
    // printf("pi2cWrite(%d, ..., %d)\n", ifd, length); fflush(stdout);
    count = 0;
    if (I2C_MULTIPROCESS_SHARING) {
	sem_wait(fdTab[ifd].sem);
	fd = fdTab[ifd].fd;
    } else {
	fd = ifd;
    }

    if (length > 127) {
        fprintf(stderr, "%s:%d: Byte write count (%d) > 127\n", __FILE__, __LINE__, length);
        goto exitPoint;
    }

    while (count < length) {
	r = write(fd, data+count, length-count);
	if (r <= 0) {
	    fprintf(stderr, "%s:%d: Failed to write device(r==%d): %s\n", __FILE__, __LINE__, r, strerror(errno));
            goto exitPoint;
	}
	count += r;
    }

exitPoint:

    if (I2C_MULTIPROCESS_SHARING) {
	sem_post(fdTab[ifd].sem);
    }

    return(count);
}

int pi2cWriteBytesToReg(int ifd, uint8_t regAddr, uint8_t length, uint8_t* data) {
    int8_t count = 0;
    uint8_t buf[128];
    
    if (length > 126) {
        fprintf(stderr, "%s:%d: Word write count (%d) > 63\n", __FILE__, __LINE__, length);
        return(0);
    }
    buf[0] = regAddr;
    memcpy(buf+1,data,length);
    count = pi2cWrite(ifd, buf, length+1);
    return(count);
}

int pi2cWriteByteToReg(int ifd, uint8_t regAddr, uint8_t data) {
    return(pi2cWriteBytesToReg(ifd, regAddr, 1, &data));
}

int pi2cWriteWordsToReg(int ifd, uint8_t regAddr, uint8_t length, uint16_t* data) {
    int8_t count = 0;
    uint8_t buf[128];
    int i;

    if (length > 63) {
        fprintf(stderr, "%s:%d: Word write count (%d) > 63\n", __FILE__, __LINE__, length);
        return(0);
    }

    buf[0] = regAddr;
    for (i = 0; i < length; i++) {
        buf[i*2+1] = data[i] >> 8;
        buf[i*2+2] = data[i];
    }
    count = pi2cWrite(ifd, buf, length*2+1);
    return(count);
}




