
#include "common.h"


// Th raspilot main data structure (shared with tools)
struct universe		*uu;
struct globalTimeInfo   currentTime;


///////////////////////////////////////////////////////////////////////////////////////////////////////////

int64_t raspilotTlibUsecTime() {
    struct timeval  tv;
    
    // TODO: Maybe move to clock_gettime(CLOCK_MONOTONIC, ...) + gettimeofday() at start
    // However in such a case the sinc with raspilot will be broken
    gettimeofday(&tv, NULL);
    return(tv.tv_sec * 1000000. + tv.tv_usec);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// pi2c

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


int pi2cReadBytesWithDelay(int ifd, uint8_t regAddr, unsigned int sleepUsec, uint8_t length, uint8_t *data) {
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
        goto exitPoint;
    }

    // optionally sleep some time between write and read
    if (sleepUsec > 0) {
	// release i2c bus while sleeping
	if (I2C_MULTIPROCESS_SHARING) sem_post(fdTab[ifd].sem);
	usleep(sleepUsec);
	if (I2C_MULTIPROCESS_SHARING) sem_wait(fdTab[ifd].sem);
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

int pi2cReadBytes(int ifd, uint8_t regAddr, uint8_t length, uint8_t *data) {
    int r;
    r = pi2cReadBytesWithDelay(ifd, regAddr, 0, length, data) ;
    return(r);
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



#if 1
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// raspilotRingBuffer (in shared memory)

void raspilotRingBufferDump(struct raspilotRingBuffer *hh) {
    int 	i, j, k, n;
    
    printf("debug %s raspilotRingBufferDump: Start.\n", hh->name);
    if (hh != NULL && hh->size != 0 && hh->vectorsize != 0) {
	n = hh->size;
	if (n > hh->n) n = hh->n;
	for(j=0; j<n; j++) {
	    i = (hh->ai + j) % hh->size;
	    printf("debug %f: ", hh->a[i*(hh->vectorsize+1)]);
	    for(k=0; k<hh->vectorsize; k++) {
		printf("%f ", hh->a[i*(hh->vectorsize+1)+k+1]);
	    }
	    printf("\n");
	}
    }
    printf("debug %s raspilotRingBufferDump: end.\n\n", hh->name);
}

void raspilotRingBufferAddElem(struct raspilotRingBuffer *hh, double time, double *vec) {
    
    // printf("%s:%d: raspilotRingBufferAddElem: %s: %f:  %s\n", __FILE__, __LINE__, hh->name, time, arrayWithDimToStr_st(vec, hh->vectorsize));
    if (hh->size == 0 || hh->vectorsize == 0) return;

    hh->a[hh->ai * (hh->vectorsize+1)] = time;
    // small optimization, when we have filled vec directly into buffer, do not copy
    if (vec != &hh->a[hh->ai*(hh->vectorsize+1)+1]) {
	memmove(&hh->a[hh->ai*(hh->vectorsize+1)+1], vec, hh->vectorsize * sizeof(double));
    }
    hh->n ++;
    hh->ailast = hh->ai;
    hh->ai = (hh->ai + 1) % hh->size;

    /*
    // for statistics
    for(i=0; i<hh->vectorsize; i++) {
    	hh->totalSumForStatistics[i] += vec[i];
	hh->totalElemsForStatistics ++;
    }
    */
}

void raspilotRingBufferInit(struct raspilotRingBuffer *hh, int vectorSize, int bufferSize, char *namefmt, ...) {
    va_list     ap;

    va_start(ap, namefmt);
    memset(hh, 0, sizeof(*hh));
    vsnprintf(hh->name, sizeof(hh->name)-1, namefmt, ap);
    assert(bufferSize > 0);
    if (bufferSize == 1) {
	printf("debug %s:%d: Warning: ring buffer %s has size %d!\n", __FILE__, __LINE__, hh->name, (bufferSize));
    }
    hh->ai = hh->ailast = 0;
    hh->n = 0;
    hh->size = bufferSize;
    hh->vectorsize = vectorSize;
    if (vectorSize >= 0) {
	memset(hh->a, 0, bufferSize*(vectorSize+1)*sizeof(double));
    }
    va_end(ap);
}

double *raspilotRingBufferGetFirstFreeVector(struct raspilotRingBuffer *hh) {
    if (hh->size == 0 || hh->vectorsize == 0) return(NULL);
    return(&hh->a[hh->ai*(hh->vectorsize+1)+1]);
}

void raspilotRingBufferFindRecordForTime(struct raspilotRingBuffer *hh, double time, double *restime, double **res) {
    int 	i, mini, maxi, ci, ri;
    double	tt;

    // find the closes record for the time, it supposes that records are ordered by time
    if (hh == NULL || hh->n < 1) {
	*res = NULL;
	return;
    }

    if (hh->n >= hh->size) {
	mini = hh->ai;
	maxi = hh->ai+hh->size-1;
    } else {
	maxi = hh->ai-1;
	mini = hh->ai-hh->n;
    }
    i = 0;
    while (maxi - mini > 1) {
	// Binary search.
	// ci = (maxi + mini) / 2;
	// Approximative search
	ci = mini + (time - hh->a[(mini%hh->size)  * (hh->vectorsize+1)]) * (maxi-mini)/(hh->a[(maxi%hh->size)  * (hh->vectorsize+1)] - hh->a[(mini%hh->size)  * (hh->vectorsize+1)]);
	// printf("<%f, %f> : %f :: <%d, %d> --> %d\n", hh->time[mini%hh->size], hh->time[maxi%hh->size], time, mini, maxi, ci);
	if (ci <= mini) ci = mini+1;
	if (ci >= maxi) ci = maxi-1;
	tt = hh->a[(ci%hh->size)  * (hh->vectorsize+1)];
	if (tt > time) {
	    maxi = ci;
	} else if (tt < time) {
	    mini = ci;
	} else {
	    ri = ci;
	    goto finito;
	}
	i++;
    }
    if (fabs(time - hh->a[(mini%hh->size)  * (hh->vectorsize+1)]) < fabs(time - hh->a[(maxi%hh->size) * (hh->vectorsize+1)])) {
	ri = mini;
    } else {
	ri = maxi;
    }
finito:
    // printf("found index after %d loops\n", i);
    if (restime != NULL) *restime = hh->a[(ri%hh->size) * (hh->vectorsize+1)];
    *res = &hh->a[(ri%hh->size)*(hh->vectorsize+1)+1];
}

/////////////////////////////////////////////////////////////////////////////////

void vec2Rotate(double *res, double *v, double theta) {
    double sint, cost, xx, yy;

    // printf("rotating %f %f by %f degree counter clockwise \n", v[0], v[1], theta*180/M_PI);
    sint = sin(theta);
    cost = cos(theta);
    // get values to local variables for case res == v
    xx = v[0];
    yy = v[1];

    if (0) {
	// clockwise rotation
	res[0] = xx * cost + yy * sint;
	res[1] = yy * cost - xx * sint;
    } else {
	// counter clockwise rotation
	res[0] = xx * cost - yy * sint;
	res[1] = yy * cost + xx * sint;
    }	
    // printf("rotated to %f %f\n", v[0], v[1]);
}

// quat to rpi and back by wiki
// not sure what is the correspondance between mpu and this
static void wikiQuaternionToEulerAngles(quat q, double *yaw, double *pitch, double *roll) {
    double x, y, z, w;
    double sinr_cosp, cosr_cosp, sinp, siny_cosp, cosy_cosp;
    
    x = q[0];
    y = q[1];
    z = q[2];
    w = q[3];
    
    // roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    *roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    // [MV] I had to change the sign here to get my pitch
    sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1) {
        *pitch = - copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
        *pitch = - asin(sinp);
    }
	
    // yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    *yaw = atan2(siny_cosp, cosy_cosp);
	
}

static void wikiEulerAnglesToQuaternion(double yaw, double pitch, double roll, quat q) {
    double cy, sy, cp, sp, cr, sr;

    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    // [MV] Use pitch with inversed sign to get back to original quaternion
    cp = cos(-pitch * 0.5);
    sp = sin(-pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    q[3] = cr * cp * cy + sr * sp * sy;
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
}

void quatToRpy(quat qq, double *roll, double *pitch, double *yaw) {
    // Experiment with those two
    // mpuQuatToYpr(qq, yaw, pitch, roll);
    wikiQuaternionToEulerAngles(qq, yaw, pitch, roll);
}

void rpyToQuat(double roll, double pitch, double yaw, quat q) {
    wikiEulerAnglesToQuaternion(yaw, pitch, roll, q);
}

void deviceSensorPositionToDronePosition(vec3 resDronePosition, vec3 sensorPosition, struct deviceData *dd, double time) {
    quat 	ii,droneOrientation;
    vec3	mm;
    double	*pose;
    double 	r,p,y;
    
    // translate from mount point to drone center of gravity
    raspilotRingBufferFindRecordForTime(uu->historyPose, time, NULL, &pose);
    if (pose == NULL) {
	// no info about orientation, suppose we are on level
	r = p = y = 0;
    } else {
	r = pose[3];
	p = pose[4];
	y = pose[5];
    }
    // TODO, do the rotation by r,p,y directly here
    rpyToQuat(r, p, y, droneOrientation);
    quat_inverse(ii, droneOrientation);
    quat_mul_vec3(mm, ii, dd->mount_position);
    vec3_add(resDronePosition, mm, sensorPosition);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: --> %s\n", PPREFIX(), vecToString_st(resDronePosition));
    vec3_sub(resDronePosition, resDronePosition, dd->mount_position);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: --> %s\n", PPREFIX(), vecToString_st(resDronePosition));
}

/////////////////////////////////////////////////////////////////////////////////
// shared memory stuff


int raspilotShmPush(struct raspilotInputBuffer *ii, double time, double *vector, int size) {
    assert(ii != NULL);
    if (ii->buffer.vectorsize != size) {
	printf("debug %s:%d: Error: %s: shared memory vector size %d does not match added vector size %d!\n", __FILE__, __LINE__, ii->buffer.name, ii->buffer.vectorsize, size);
	return(-2);
    }
    if (ii->status == RIBS_SHARED_FINALIZE) {
	return(-1);
    }
    pthread_mutex_lock(&ii->mutex);
    raspilotRingBufferAddElem(&ii->buffer, time, vector);
    msync(ii, RASPILOT_INPUT_BUFFER_SIZE(ii->buffer.size, ii->buffer.vectorsize), MS_SYNC);
    pthread_mutex_unlock(&ii->mutex);
    return(0);
}	 

struct raspilotInputBuffer *raspilotShmConnect(char *name) {
    int 			fd;
    int				r, len, size, vectorsize;
    struct raspilotInputBuffer 	*res;
    pthread_mutexattr_t 	attr;

    fd = -1;
    res = NULL;
    
    fd = shm_open(name, O_RDWR, S_IRWXU);
    if (fd == -1) {
	// We cannot use lprintf here as this function is used in device module code
	printf("debug %s:%d: Error: Can't connect to shared memory %s\n", __FILE__, __LINE__, name);
	goto failexitpoint;
    }
    len = sizeof(struct raspilotInputBuffer);
    res = (struct raspilotInputBuffer *) mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (res == NULL) {
	printf("debug %s:%d: Error: Can't mmap shared memory %s\n", __FILE__, __LINE__, name);
	goto failexitpoint;
    }
    __sync_synchronize();
    if (res->magicVersion != RASPILOT_SHM_MAGIC_VERSION) {
	usleep(10000);
	__sync_synchronize();
	if (res->magicVersion != RASPILOT_SHM_MAGIC_VERSION) {
	    printf("debug %s:%d: Error: Wrong version code in shared memory %s\n", __FILE__, __LINE__, name);
	    goto failexitpoint;
	}
    }
    if (res->status != RIBS_SHARED_INITIALIZE) {
	printf("debug %s:%d: Error: Wrong initial status %d in shared memory %s\n", __FILE__, __LINE__, res->status, name);
	goto failexitpoint;
    }

    vectorsize = res->buffer.vectorsize;
    size = res->buffer.size;
    // ok, we have everything we need to get the actual length, remap the memory
    munmap(res, len);
    len = RASPILOT_INPUT_BUFFER_SIZE(size, vectorsize);
    res = (struct raspilotInputBuffer *) mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (res == NULL) {
	printf("debug %s:%d: Error: Can't re-mmap shared memory of %s\n", __FILE__, __LINE__, name);
	goto failexitpoint;
    }

    r = pthread_mutexattr_init(&attr);
    if (r != 0) {
	printf("debug %s:%d: Error: Can't get mutex attributes of %s\n", __FILE__, __LINE__, name);
	goto failexitpoint;
    }
    r = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    if (r != 0) {
	printf("debug %s:%d: Error: Can't set mutex attributes of %s\n", __FILE__, __LINE__, name);
	goto failexitpoint;
    }
    r = pthread_mutex_init(&res->mutex, &attr);
    if (r != 0) {
	printf("debug %s:%d: Error: Can't init mutex of %s\n", __FILE__, __LINE__, name);
	goto failexitpoint;
    }
    raspilotRingBufferInit(&res->buffer, vectorsize, size, (char*)"%s shm stream", name);

    res->status = RIBS_SHARED_OK;
    // according to doc, we can close fd now.
    close(fd);
    return(res);


failexitpoint:
    if (fd >= 0) close(fd);
    if (res != NULL) munmap(res, len);
    return(NULL);
}

char *raspilotShmConnectToUniverse(char *name, int size, int version, int *errCode) {
    int 				fd;
    struct raspilotUniversePrefix	*res;
    char				*addr;
    
    fd = -1;
    res = NULL;

    fd = shm_open(name, O_RDWR, 0);
    if (fd == -1) {
	// We cannot use lprintf here as this function is used in device module code
	printf("debug %s:%d: Error: Can't connect to shared memory %s\n", __FILE__, __LINE__, name);
	*errCode = 4;
	goto failexitpoint;
    }
    res = (struct raspilotUniversePrefix *) mmap(NULL, size, PROT_READ, MAP_SHARED, fd, 0);
    if (res == NULL) {
	printf("debug %s:%d: Error: Can't mmap shared memory %s\n", __FILE__, __LINE__, name);
	*errCode = 3;
	goto failexitpoint;
    }
    // __sync_synchronize();
    if (res->version != version) {
	printf("%s:%d: Error: Wrong version %d in shared %s, expected %d\n", __FILE__, __LINE__, res->version, name, version);
	printf("debug %s:%d: Error: Wrong version in shared %s\n", __FILE__, __LINE__, name);
	*errCode = 2;
	goto failexitpoint;
    }

    addr = res->self;
    munmap(res, size);
    // printf("debug %s:%d: Remapping from %p to %p\n", __FILE__, __LINE__, res, addr);fflush(stdout);
    // If I put MAP_FIXED it makes sigsegv !!! So do it without and hope for the best.
    // BTW, 64bit OS is very much recommended due to this. It works much better there.
    res = (struct raspilotUniversePrefix *) mmap(addr, size, PROT_READ, MAP_SHARED, fd, 0);
    // printf("debug %s:%d: Got remapped to %p\n", __FILE__, __LINE__, res);fflush(stdout);
    if (res == NULL || (char*)res != addr) {
	printf("debug %s:%d: Warning: Problem with re-mmaping of %s\n", __FILE__, __LINE__, name);
	*errCode = 1;
	goto failexitpoint;
    }
    close(fd);
    *errCode = 0;
    return((char*)res);

failexitpoint:
    fflush(stdout);
    if (fd >= 0) close(fd);
    if (res != NULL) munmap(res, size);
    return(NULL);
}

// Restart a program. It is used in case when the universe mapping fails hoping that the next time it will work.
void raspilotShmRestartProgram(int argc, char **argv, int sleepUs) {
    char *nullargv[argc+1];
    int  i;
    
    usleep(sleepUs);
    for(i=0; i<argc; i++) nullargv[i] = argv[i];
    nullargv[i] = NULL;
    execv(argv[0], nullargv);
}

#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// misc

static char baioStaticStringsRing[STATIC_STRINGS_RING_SIZE][TMP_STRING_SIZE];
static int  baioStaticStringsRingIndex = 0;

char *getTemporaryStringPtrFromStaticStringRing() {
    char *res;
    
    res = baioStaticStringsRing[baioStaticStringsRingIndex];
    baioStaticStringsRingIndex  = (baioStaticStringsRingIndex+1) % STATIC_STRINGS_RING_SIZE;
    // make sure that snprint-ed string will be zero terminating
    res[TMP_STRING_SIZE-1] = 0;
    return(res);
}

char *printPrefix_st(struct universe *uu, char *file, int line) {
    int		i, r;
    char	*res;

    // Hmm. BTW this is a costly function if there is a lot of debug output. Maybe optimized a bit.
    res = getTemporaryStringPtrFromStaticStringRing();
    r = snprintf(res, TMP_STRING_SIZE-1, "%s: %s:%d", currentLocalTime_st(), file, line);
    for(i=r; i>=0 && i<40; i++) res[i] = ' ';
    res[i] = 0;
    return(res);
}

char *currentLocalTime_st() {
    char            *res;

    res = getTemporaryStringPtrFromStaticStringRing();

    snprintf(res, TMP_STRING_SIZE-1, "%4d-%02d-%02d %02d:%02d:%02d.%03d", 
             1900+currentTime.lcltm.tm_year, currentTime.lcltm.tm_mon+1, currentTime.lcltm.tm_mday, 
             currentTime.lcltm.tm_hour, currentTime.lcltm.tm_min, currentTime.lcltm.tm_sec,
             currentTime.msecPart);
    return(res);
}

char *sprintSecTime_st(long long int utime) {
    static char     *res;
    time_t          t;
    struct tm       *tm, ttm;

    res = getTemporaryStringPtrFromStaticStringRing();
    t = utime / 1000000;
#if _WIN32
    ttm = *localtime(&t);
    tm = &ttm;
#else
    tm = localtime_r(&t, &ttm);
#endif
    snprintf(res, TMP_STRING_SIZE-1, "%4d-%02d-%02d %02d:%02d:%02d", 
	     1900+tm->tm_year, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec
	);
    return(res);
}

char *sprintUsecTime_st(long long int utime) {
    char     		*res;
    time_t          	t;
    int             	u;
    struct tm       	*tm, ttm;

    res = getTemporaryStringPtrFromStaticStringRing();
    t = utime / 1000000;
    u = utime % 1000000;
#if _WIN32
    ttm = *localtime(&t);
    tm = &ttm;
#else
    tm =  localtime_r(&t, &ttm);
#endif
    snprintf(res, TMP_STRING_SIZE-1, "%4d-%02d-%02d %02d:%02d:%02d.%03d %03d", 
	     1900+tm->tm_year, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec,
	     u/1000, u%1000);
    return(res);
}

void setCurrentTimeToTimeVal(struct timeval *tv) {
    int             previousTimeHour, m, s;

    if (currentTime.sec < tv->tv_sec || (currentTime.sec == tv->tv_sec && currentTime.usecPart < tv->tv_usec)) {

        previousTimeHour = currentTime.hour;

        // update current time
        currentTime.sec = tv->tv_sec;
        currentTime.hour = tv->tv_sec / (60*60);
        currentTime.usecPart = tv->tv_usec;
        currentTime.msecPart = tv->tv_usec / 1000;
        currentTime.usec = ((long long int)tv->tv_sec) * 1000000LL + tv->tv_usec;
        currentTime.dtime = tv->tv_sec + tv->tv_usec / 1000000.0;
        // currentTime.dtime = (tv->tv_sec - 1640995200) + tv->tv_usec / 1000000.0;	// like this since 1.1.2022
	currentTime.msec = currentTime.usec / 1000;
	//currentTime.msec = currentTime.dtime * 1000.0;
	
	// update tm structures
	// TODO: check if we are the same halfhour, some timezones are half an hour shifted
        if (currentTime.hour == previousTimeHour) {
            // we are the same hour as previously, no need to call localtime, update only minutes and seconds in tm structures
            s = currentTime.sec % 60;
            m = (currentTime.sec / 60) % 60;
            currentTime.gmttm.tm_sec = currentTime.lcltm.tm_sec = s;
            currentTime.gmttm.tm_min = currentTime.lcltm.tm_min = m;
        } else {
#if _WIN32
	    currentTime.gmttm = *gmtime(&currentTime.sec);
            currentTime.lcltm = *localtime(&currentTime.sec);
#else
            gmtime_r(&currentTime.sec, &currentTime.gmttm);
            localtime_r(&currentTime.sec, &currentTime.lcltm);
#endif
        }
    }
}

void setCurrentTime() {
    struct timeval  tv;

    // TODO: Maybe move to clock_gettime(CLOCK_MONOTONIC_RAW, ...)
    gettimeofday(&tv, NULL);
    setCurrentTimeToTimeVal(&tv);
}

void incrementCurrentTime() {
    struct timeval  tv;	

    tv.tv_sec = currentTime.sec;
    tv.tv_usec = currentTime.usecPart;
    tv.tv_usec ++;
    if (tv.tv_usec >= 1000000) {
	tv.tv_sec += tv.tv_usec / 1000000;
	tv.tv_usec = tv.tv_usec % 1000000;		
    }
    setCurrentTimeToTimeVal(&tv);
}

struct deviceData *deviceFindByName(char *name) {
    int i;
    if (uu == NULL || name == NULL) return(NULL);
    for(i=0; i<uu->deviceMax; i++) {
	if (strcmp(uu->device[i]->name, name) == 0) return(uu->device[i]);
    }
    return(NULL);
}

struct deviceStreamData *deviceFindStreamByName(struct deviceData *dd, char *name) {
    int i;
    if (dd == NULL || name == NULL) return(NULL);
    for(i=0; i<dd->ddtMax; i++) {
	if (strcmp(dd->ddt[i]->name, name) == 0) return(dd->ddt[i]);
    }
    return(NULL);
}

struct deviceStreamData *deviceFindStreamByType(struct deviceData *dd, int type) {
    int i;
    if (dd == NULL) return(NULL);
    for(i=0; i<dd->ddtMax; i++) {
	if (dd->ddt[i]->type == type) return(dd->ddt[i]);
    }
    return(NULL);
}


char *arrayWithDimToStr_st(double *a, int dim) {
    char	*res, *separator;
    int		i, j;
    
    res = getTemporaryStringPtrFromStaticStringRing();
    i = 0;
    if (i>=TMP_STRING_SIZE-1) return((char*)(FILE_LINE_ID_STR() ": Error"));
    i += snprintf(res+i, TMP_STRING_SIZE-i-1, "[");
    separator = (char*)"";
    for(j=0; j<dim; j++) {
	if (i>=TMP_STRING_SIZE-1) return((char*)"Error: vector too large to print");
	i += snprintf(res+i, TMP_STRING_SIZE-i-1, "%s%7.3f", separator, a[j]);
	// i += snprintf(res+i, TMP_STRING_SIZE-i-1, "%s%9.5f", separator, a[j]);
	separator = (char*)" ";
    }
    if (i>=TMP_STRING_SIZE-1) return((char*)"Error: vector too large to print");
    i += snprintf(res+i, TMP_STRING_SIZE-i-1, "]");
    return(res);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// tool library stuff

struct raspilotTlibStr *raspilotTlibInit(struct raspilotTlibStr *tt, int argc, char **argv, int mapRaspilotUniverseFlag) {
    int 		i;
    struct universe 	*uuu;
    
    // the very first thing is to map raspilot universe
    uuu = NULL;
    if (mapRaspilotUniverseFlag) {
	CONNECT_TO_RASPILOT_UNIVERSE(argc, argv, 10000, uuu);
    }
    
    // init the structure and set tlib link to universe
    memset(tt, 0, sizeof(*tt));
    tt->universe = uu = uuu;
    
    // create a copy of arguments
    tt->argc = argc;
    tt->argv = (char**) malloc(argc+1*sizeof(char*));
    for(i=0; i<argc; i++) tt->argv[i] = strdup(argv[i]);
    tt->argv[i] = NULL;

    // small hack, continue working with my copy of argumens
    argv = tt->argv;
    
    // set default values
    tt->optSharedI2cFlag = 0;
    tt->optI2cPath = (char*)"/dev/i2c-1";
    tt->deviceName = NULL;
    tt->raspilotReadTimeoutUsec = 1000000;
    raspilotTlibSetRefreshRateHz(tt, 100.0);
    
    // Get some options from environment variables, if they are there.
    // Raspilot is supposed to prepare something
    //
    tt->deviceName = getenv("RP_DEVICE_NAME");
    if (getenv("RP_I2C_SHARED") != NULL) tt->optSharedI2cFlag = 1;
    
    // go through arguments
    for(i=0; i<argc; i++) {
	if (strcmp(argv[i], "-r") == 0) {
	    // refresh rate
	    i++;
	    if (i<argc) raspilotTlibSetRefreshRateHz(tt, strtod(argv[i], NULL));
	} else if (strcmp(argv[i], "-i2cs") == 0) {
	    // share i2c. Do not reset shared semaphores
	    tt->optSharedI2cFlag = 1;
	} else if (strcmp(argv[i], "-i2c") == 0) {
	    // path to i2c device
	    i++;
	    if (i<argc) tt->optI2cPath = argv[i];
	} else if (strcmp(argv[i], "-shm") == 0 || strcmp(argv[i], "-rp_device_name") == 0) {
	    i++;
	    if (i<argc) tt->deviceName = argv[i];
	}
    }

    // find my device data
    tt->dd = deviceFindByName(tt->deviceName);
    
    // init for refresh
    tt->lastSampleTimeUsec = raspilotTlibUsecTime();
    return(tt);
}

int raspilotTlibInitI2cDevice(struct raspilotTlibStr *tt, int address) {
    int i;

    i = tt->i2ci;
    if (i >= RASPILOT_TLIB_I2C_DEV_MAX) {
	printf("debug Error: too many i2c devices\n"); fflush(stdout);
	return(-1);
    }
    if (tt->optSharedI2cFlag) pi2cInit(tt->optI2cPath, tt->optSharedI2cFlag);
    tt->i2ci++;
    tt->i2c[i] = pi2cOpen(tt->optI2cPath, address);
    return(i);
}

////

void raspilotTlibOnDisconnection(struct raspilotTlibStr *tt, struct raspilotTlibStream *ss) {
    char *file;

    file = strrchr((char*)__FILE__, '/');
    if (file == NULL) {
	file = (char*)__FILE__;
    } else {
	file ++;
    }
    
    fprintf(stderr, "!%s:%d: %s: Error: %s disconnected!\n", file, __LINE__, tt->deviceName, ss->tag);
    if (tt->onRaspilotDisconnection == NULL) {
	fprintf(stderr, "!%s:%d: %s: Exiting!\n", file, __LINE__, tt->deviceName);
	exit(-1);
    }
    tt->onRaspilotDisconnection(tt, ss);
}

////

void raspilotTlibSetRefreshRateHz(struct raspilotTlibStr *tt, double refreshRateHz) {
    tt->sampleRate = refreshRateHz;
    tt->sleepTimeUsec = tt->requiredPeriodUsec = 1000000 / refreshRateHz;
}

void raspilotTlibMainLoopSleep(struct raspilotTlibStr *tt, int64_t sampleTimeUsec) {
    int64_t samplePeriodUsec;

    // if we are not SHM connection do fflush(stdout) for case the user have forgotten it
    if (tt->deviceName == NULL) fflush(stdout);
    
    if (sampleTimeUsec <= 0) sampleTimeUsec = raspilotTlibUsecTime();
    samplePeriodUsec = sampleTimeUsec - tt->lastSampleTimeUsec;
    tt->lastSampleTimeUsec = sampleTimeUsec;
    if (samplePeriodUsec > tt->requiredPeriodUsec && tt->sleepTimeUsec > 0) tt->sleepTimeUsec --;
    else if (samplePeriodUsec < tt->requiredPeriodUsec) tt->sleepTimeUsec ++;
    usleep(tt->sleepTimeUsec);
}

char *raspilotDeviceStreamSharedMemName_st(char *deviceName, char *streamTag) {
    char *res;

    res = getTemporaryStringPtrFromStaticStringRing();
    snprintf(res, TMP_STRING_SIZE-1, "raspilot.%s.%s", deviceName, streamTag);
    return(res);
}

int raspilotTlibInitStream(struct raspilotTlibStr *tt, char *streamTag, int sharedMemoryFlag) {
    int 			i, r;
    struct raspilotTlibStream	*ss;
    char			*shmName;
    
    i = tt->streami;
    if (i >= RASPILOT_TLIB_STREAMS_MAX) {
	printf("debug Error: too many streams\n"); fflush(stdout);
	return(-1);
    }
    ss = &tt->stream[i];
    ss->tag = streamTag;
    ss->shmModeActive = 0;
    ss->shmbuf = NULL;
    if (sharedMemoryFlag) {
	if (tt->deviceName == NULL) {
	    printf("debug Warning: no shm device name set. Stream %s can not use shared memory!\n", streamTag);
	    fflush(stdout);
	} else {
	    shmName = raspilotDeviceStreamSharedMemName_st(tt->deviceName, streamTag);
	    ss->shmbuf = raspilotShmConnect(shmName);
	    if (ss->shmbuf == NULL) {
		printf("debug Error: stream %s.%s can't use shared memory!\n", tt->deviceName, streamTag);
		fflush(stdout);
	    } else {
		ss->shmModeActive = 1;
	    }
	}
    }
    tt->streami ++;
    return(i);
}

int raspilotTlibSend(struct raspilotTlibStr *tt, int streamIndex, int64_t sampleTimeUsec, double confidence, double *vector, int vectorLength) {
    struct raspilotTlibStream	*ss;
    int				i, r;
    
    if (streamIndex < 0) return(-1);
    
    ss = &tt->stream[streamIndex];
    if (ss->shmModeActive) {
	if (sampleTimeUsec <= 0) sampleTimeUsec = raspilotTlibUsecTime();
	// If raspilot changed the status of the shared memory, it was shut down
	if (ss->shmbuf->status != RIBS_SHARED_OK) raspilotTlibOnDisconnection(tt, ss);
	if (tt->raspilotReadTimeoutUsec != 0 && ss->shmbuf->lastReadTimeUsec != 0) {
	    // if raspilot does not read the stream for a long time, consider it dosconnected
	    if (ss->shmbuf->lastReadTimeUsec < sampleTimeUsec - tt->raspilotReadTimeoutUsec) raspilotTlibOnDisconnection(tt, ss);
	}
	ss->shmbuf->confidence = confidence;
	r = raspilotShmPush(ss->shmbuf, sampleTimeUsec/1000000.0, vector, vectorLength);
	return(r);
    } else {
	printf("%s ", ss->tag);
	for(i=0; i<vectorLength; i++) printf("%f ", vector[i]);
	printf("\n");
	r = fflush(stdout);
	if (r != 0) raspilotTlibOnDisconnection(tt, ss);
    }
    return(r);
}
