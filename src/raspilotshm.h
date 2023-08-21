#ifndef _RASPILOTSHM__H_
#define _RASPILOTSHM__H_ 1


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h> 


#define RASPILOT_SHM_MAGIC_VERSION 			0xcfe0234
#define RASPILOT_RING_BUFFER_SIZE(size, vectorsize) 	(sizeof(struct raspilotRingBuffer) + (size) * ((vectorsize)+1) * sizeof(double))
#define RASPILOT_INPUT_BUFFER_SIZE(size, vectorsize) 	(sizeof(struct raspilotInputBuffer) + (size) * ((vectorsize)+1) * sizeof(double))

enum raspilotInpuBufferStatusEnum {
    RIBS_NONE,
    RIBS_NOT_SHARED,
    RIBS_SHARED_INITIALIZE,
    RIBS_SHARED_OK,
    RIBS_SHARED_FINALIZE,
    RIBS_MAX,
};

// Ring buffer storing vectors is used to pass values from devices to raspilot
struct raspilotRingBuffer {
    char		name[256];	// for debug output only?
    int			size;
    int			vectorsize;
    int			ai;		// index where next elem will be stored
    int			ailast;		// index where the last elem was added, i.e.  (ai-1) % size
    int			n;		// number of total inserted elements, not only currently stored (ai == n%size)

    // the actual data stored in the buffer are allocate after the structure
    double		a[0];		// a[size][vectorsize+1] // a[i][0] is time/key, then goes the vector
};

struct raspilotInputBuffer {
    // Raw data coming from the sensor are parsed, "timestamped" and put into this buffer.
    // Devices putting values through shared memory write directly to this buffer and initialize mutex.
    // Mutex is not initialized/activated if status == RIBS_NOT_SHARED
    enum raspilotInpuBufferStatusEnum	status;
    pthread_mutex_t 			mutex;
    // buffer must be the last member of the struct, because its data are allocated after it.
    int					magicVersion;	// some number to verifying that both raspilot and device are using the same version of shm
    // must be the last, actual buffer is allocated after this structure
    struct raspilotRingBuffer		buffer;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline void raspilotRingBufferAddElem(struct raspilotRingBuffer *hh, double time, double *vec) {
    int 	i;
    
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

static inline void raspilotRingBufferInit(struct raspilotRingBuffer *hh, int vectorSize, int bufferSize, char *namefmt, ...) {
    int		i;
    va_list     ap;

    va_start(ap, namefmt);
    memset(hh, 0, sizeof(*hh));
    vsnprintf(hh->name, sizeof(hh->name)-1, namefmt, ap);
    assert(bufferSize > 0);
    if (bufferSize == 1) {
	printf("%s:%d: Warning: ring buffer %s has size %d!\n", __FILE__, __LINE__, hh->name, (bufferSize));
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

static inline void raspilotShmPush(struct raspilotInputBuffer *ii, double time, double *vector) {
    pthread_mutex_lock(&ii->mutex);
    raspilotRingBufferAddElem(&ii->buffer, time, vector);
    pthread_mutex_unlock(&ii->mutex);
    //? msync(ii, RASPILOT_INPUT_BUFFER_SIZE(ii->buffer.size, ii->buffer.vectorsize), MS_SYNC);
}	 

static inline struct raspilotInputBuffer *raspilotShmConnect(char *name) {
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
    res = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
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
    res = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
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
    raspilotRingBufferInit(&res->buffer, vectorsize, size, "%s shm stream", name);

    res->status = RIBS_SHARED_OK;
    // according to doc, we can close fd now.
    close(fd);
    return(res);


failexitpoint:
    if (fd >= 0) close(fd);
    if (res != NULL) munmap(res, len);
    return(NULL);
}



#endif
