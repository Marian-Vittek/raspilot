//////////////////////////////////////////////////////////////////////////////////////////////////
// this file implements a library allowing assynchronous read/write
// baio stands for Basic Assynchronous Input Output

#include "common.h"


#define BAIO_STATUSES_CLEARED_PER_TICK              0x0ffffff0


struct baio 	*baioTab[BAIO_MAX_CONNECTIONS];
int 		baioTabMax;

int 		baioDebugLevel = 10;

//////////////////////////////////////////////////////////////////////////////////////////////////

void baioCharBufferDump(char *prefix, char *s, int n) {
    int i;
    printf("%s", prefix);
    for(i=0; i<n; i++) printf(" %02x", ((unsigned char*)s)[i]);
    printf("\n");
    fflush(stdout);
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// callbacks

void callBackClearHook(struct callBackHook *h) {
    h->i = 0;
}

void jsVarCallBackFreeHook(struct callBackHook *h) {
    h->i = 0;
    FREE(h->a);
    h->a = NULL;
    h->dim = 0;
}

int callBackAddToHook(struct callBackHook *h, callBackHookFunArgType ptr) {
    int i;

    i = h->i;
    if (i >= h->dim) {
        h->dim = h->dim * 2 + 1;
        REALLOCC(h->a, h->dim, callBackHookFunType);
    }
    h->a[i] = (callBackHookFunType) ptr;
    h->i ++;
    return(i);
}

static void jsVarCallBackRemoveIndexFromHook(struct callBackHook *h, int i) {
    if (i < 0 || i >= h->i) return;
    for(i=i+1; i < h->i; i++) h->a[i-1] = h->a[i];
    h->i --;
}

void callBackRemoveFromHook(struct callBackHook *h, callBackHookFunArgType ptr) {
    int i;

    for(i=0; i<h->i && h->a[i] != ptr; i++) ;
    if (i < h->i) {
        jsVarCallBackRemoveIndexFromHook(h, i);
    }
}

// if src is NULL, allocate new copy of src itself
void jsVarCallBackCloneHook(struct callBackHook *dst, struct callBackHook *src) {
    callBackHookFunType *a;

    if (src != NULL) *dst = *src;
    if (dst->dim != 0) {
	a = dst->a;
	ALLOCC(dst->a, dst->dim, callBackHookFunType);
	memcpy(dst->a, a, dst->dim*sizeof(callBackHookFunType));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////

int setFileNonBlocking(int fd) {
#if 0 || _WIN32
    return(-1);
#else
    int    flg;

    if (fd < 0) return(-1);
    // Set non-blocking 
    if((flg = fcntl(fd, F_GETFL, NULL)) < 0) { 
        printf("%s: %s:%d: Error on socket %d fcntl(..., F_GETFL) (%s)\n", PPREFIX(), __FILE__, __LINE__, fd, strerror(errno)); 
        return(-1);
    } 
    flg |= O_NONBLOCK; 
    if(fcntl(fd, F_SETFL, flg) < 0) { 
        printf("%s: %s:%d: Error on socket %d fcntl(..., F_SETFL) (%s)\n", PPREFIX(), __FILE__, __LINE__, fd, strerror(errno)); 
        return(-1);
    }
    return(0);
#endif
}

int setFileBlocking(int fd) {
#if _WIN32
    return(-1);
#else
    int    flg;
    // Set blocking 
    if((flg = fcntl(fd, F_GETFL, NULL)) < 0) { 
        printf("%s: %s:%d: Error on socket %d fcntl(..., F_GETFL) (%s)\n", PPREFIX(), __FILE__, __LINE__, fd, strerror(errno)); 
        return(-1);
    } 
    flg &= ~(O_NONBLOCK); 
    if(fcntl(fd, F_SETFL, flg) < 0) { 
        printf("%s: %s:%d: Error on socket %d fcntl(..., F_SETFL) (%s)\n", PPREFIX(), __FILE__, __LINE__, fd, strerror(errno)); 
        return(-1);
    }
    return(0);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////


static void baioBufferCreate(struct baioBuffer *b, int size) {
    ALLOCC(b->b, size, char);
    b->i = b->j = 0;
    b->size = size;
}

static void baioBufferFree(struct baioBuffer *b) {
    FREE(b->b);
    b->b = NULL;
    b->i = b->j = 0;
    b->size = 0;
}

static int baioBufferShift(struct baio *bb, struct baioBuffer *b) {
    int         delta;

    assert(b->i <= b->j);
    delta = b->i;
    if (delta == 0) return(delta);
    memmove(b->b, b->b+delta, (b->j - delta) * sizeof(b->b[0]));
    b->j -= delta;
    b->i -= delta;
    CALLBACK_CALL(bb->callBackOnBufferShift, callBack(bb));
    return(delta);
}

static void baioFreeZombie(struct baio *bb) {
    int i;

    i = bb->index;
    assert(bb == baioTab[i]);

    if (bb->readBuffer.i != bb->readBuffer.j) {
        printf("%s: Warning: zombie read.\n", PPREFIX());
    }
    if (bb->writeBuffer.i != bb->writeBuffer.j) {
        printf("%s: Warning: zombie write.\n", PPREFIX());
    }
    baioBufferFree(&bb->readBuffer);
    baioBufferFree(&bb->writeBuffer);

    // free all hooks
    jsVarCallBackFreeHook(&bb->callBackOnRead);
    jsVarCallBackFreeHook(&bb->callBackOnWrite);
    jsVarCallBackFreeHook(&bb->callBackOnError);
    jsVarCallBackFreeHook(&bb->callBackOnEof);
    jsVarCallBackFreeHook(&bb->callBackOnDelete);
    jsVarCallBackFreeHook(&bb->callBackOnBufferShift);

    FREE(bb); 
    baioTab[i] = NULL;
    if (i == baioTabMax - 1) baioTabMax --;
}

void baioCloseFd(struct baio *bb) {
    if (bb->rfd >= 0) close(bb->rfd);
    if (bb->wfd >= 0 && bb->wfd != bb->rfd) close(bb->wfd);
}

static int baioImmediateDeactivate(struct baio *bb) {
    // zombie will be cleared at the next cycle
    bb->status = BAIO_STATUS_ZOMBIE;
    bb->baioMagic = 0;
    // soft clean buffers
    bb->readBuffer.i = bb->readBuffer.j = 0;
    bb->writeBuffer.i = bb->writeBuffer.j = 0;

    if (bb->rfd < 0 && bb->wfd < 0) return(-1);
    if (bb->baioType != BAIO_TYPE_FD) {
        baioCloseFd(bb);
    }
    // we have to call callback before reseting fd to -1, because delete call back may need it
    CALLBACK_CALL(bb->callBackOnDelete, callBack(bb));
    bb->rfd = -1;
    bb->wfd = -1;

    return(0);
}

int baioCloseOnError(struct baio *bb) {
    CALLBACK_CALL(bb->callBackOnError, callBack(bb));
    baioImmediateDeactivate(bb);
    return(-1);
}

struct baio *baioFromMagic(int baioMagic) {
    int             i;
    struct baio     *bb;
    // baioMagic can never be 0
    if (baioMagic == 0) return(NULL);
    i = (baioMagic % BAIO_MAX_CONNECTIONS);
    bb = baioTab[i];
    if (bb == NULL) return(NULL);
    if (bb->baioMagic != baioMagic) return(NULL);
    if ((bb->status & BAIO_STATUS_ACTIVE) == 0) return(NULL);
    if ((bb->status & BAIO_STATUS_PENDING_CLOSE) != 0) return(NULL);
    return(bb);
}

/////////////////////////////////////////////////////////////////////////////////

static int baioTabFindUnusedEntryIndex(int baioStructType) {
    int i;

    for(i=0; i<baioTabMax && baioTab[i] != NULL; i++) ;

    if (i >= baioTabMax) {
        if (i >= BAIO_MAX_CONNECTIONS) {
            printf("%s: %s:%d: Error: Can't allocate baio. Too many connections\n", PPREFIX(), __FILE__, __LINE__);
            return(-1);
        }
        baioTabMax = i+1;
    }
    return(i);
}

int baioLibraryInit(int deInitializationFlag) {
    static int      libraryInitialized = 0;
    int             r;

    if (libraryInitialized == 0 && deInitializationFlag == 0) {
        libraryInitialized = 1;
    } else if (libraryInitialized != 0 && deInitializationFlag != 0) {
        libraryInitialized = 0;
    } else {
        return(-1);
    }
    return(0);
}

static struct baio *baioInitBasicStructure(int i, int baioType, int ioDirections, int additionalSpaceAllocated) {
    struct baio *bb;

    bb = baioTab[i];
    assert(bb != NULL);
    memset(bb, 0, sizeof(struct baio)+additionalSpaceAllocated);
    bb->initialReadBufferSize = (1<<10);
    bb->initialWriteBufferSize = (1<<10);
    bb->minFreeSizeBeforeRead = 8;
    bb->minFreeSizeAtEndOfReadBuffer = 1;
    bb->maxInactivityTime = BAIO_MAX_INACTIVITY_TIME_INFINITY;

    bb->ioDirections = ioDirections;
    bb->status = BAIO_STATUS_ACTIVE;
    bb->rfd = -1;
    bb->wfd = -1;
    bb->index = i;

    // baiomagic shall never be zero 
    // bb->baioMagic = (((magicCounter++) & 0x1ffffe) + 1) * BAIO_MAX_CONNECTIONS + i;
    // Keep in mind that BAIO_MAX_CONNECTIONS may be arbitrary large
    // If the following assertion fail, adjust the constant accordingly
    assert(INT_MAX / BAIO_MAX_CONNECTIONS > BAIO_MAGIC_MASK);
    bb->baioMagic = ((rand() & BAIO_MAGIC_MASK) + 1) * BAIO_MAX_CONNECTIONS + i;
    assert(bb->baioMagic > 0);

    bb->baioType = baioType;
    bb->additionalSpaceAllocated = additionalSpaceAllocated;
    bb->lastActivityTime = time(NULL);

    memset(&bb->readBuffer, 0, sizeof(bb->readBuffer));
    memset(&bb->writeBuffer, 0, sizeof(bb->writeBuffer));
    return(bb);
}

struct baio *baioNewBasic(int baioType, int ioDirections, int additionalSpaceToAllocate) {
    int             i;
    struct baio     *bb;

    baioLibraryInit(0);
    i = baioTabFindUnusedEntryIndex(baioType);
    if (i < 0) return(NULL);
    assert(baioTab[i] == NULL);
    if (baioTab[i] == NULL) ALLOC_SIZE(baioTab[i], struct baio, sizeof(struct baio)+additionalSpaceToAllocate);
    bb = baioInitBasicStructure(i, baioType, ioDirections, additionalSpaceToAllocate);
    return(bb);
}

int baioClose(struct baio *bb) {
    if (bb == NULL) return(-1);
    if (bb->ioDirections == BAIO_IO_DIRECTION_READ || bb->ioDirections == BAIO_IO_DIRECTION_RW) {
        baioBufferFree(&bb->readBuffer);
    }
    if (bb->ioDirections == BAIO_IO_DIRECTION_WRITE || bb->ioDirections == BAIO_IO_DIRECTION_RW) {
        if (bb->writeBuffer.i != bb->writeBuffer.j) {
            bb->status |= BAIO_STATUS_PENDING_CLOSE;
        }
    }
    if ((bb->status & BAIO_STATUS_PENDING_CLOSE) == 0) {
        baioImmediateDeactivate(bb);
        return(0);
    }
    return(1);
}

int baioCloseMagic(int baioMagic) {
    struct baio *bb;
    int         res;
    bb = baioFromMagic(baioMagic);
    if (bb == NULL) return(-1);
    res = baioClose(bb);
    return(res);
}

int baioCloseMagicOnError(int baioMagic) {
    struct baio *bb;
    int         res;
    bb = baioFromMagic(baioMagic);
    if (bb == NULL) return(-1);
    res = baioCloseOnError(bb);
    return(res);
}

void baioPurgeInactiveConnections(int maxCheckedItems) {
    static int 		i = 0;
    int			j;
    time_t		ctime;
    struct baio *bb;
	
    ctime = time(NULL);
    if (maxCheckedItems > baioTabMax) maxCheckedItems = baioTabMax;

    for(j=0; j<maxCheckedItems; j++) {
	i = (i+1) % baioTabMax;
	bb = baioTab[i];
	if (bb != NULL 
	    && bb->status != BAIO_STATUS_ZOMBIE
	    && ctime - bb->lastActivityTime > bb->maxInactivityTime 
	    && bb->maxInactivityTime != BAIO_MAX_INACTIVITY_TIME_INFINITY
	    ) {
	    baioImmediateDeactivate(bb);
	}
    }
}

static int baioSizeOfContinuousFreeSpaceForWrite(struct baioBuffer *b) {
    return(b->size - b->j);
}

static int baioGetSpaceForWrite(struct baio *bb, struct baioBuffer *b, int n) {
    int                     r;

    if (b->b == NULL) baioBufferCreate(b, bb->initialWriteBufferSize);
    if (baioSizeOfContinuousFreeSpaceForWrite(b) >= n) return(0);
    baioBufferShift(bb, b);
    if (baioSizeOfContinuousFreeSpaceForWrite(b) >= n) return(0);
    return(-1);
}

int baioWriteToBuffer(struct baio *bb, char *s, int len) {
    int                     r;
    char                    *d;
    struct baioBuffer  *b;

    if ((bb->status & BAIO_STATUS_ACTIVE) == 0) return(-1);

    r = baioGetSpaceForWrite(bb, &bb->writeBuffer, len);
    if (r < 0) return(-1);
    b = &bb->writeBuffer;
    d = b->b + b->j;
    memmove(d, s, len);
    b->j += len;
    return(len);
}

int baioVprintfToBuffer(struct baio *bb, char *fmt, va_list arg_ptr) {
    int                     n, r, dsize;
    struct baioBuffer  *b;
    va_list                 arg_ptr_copy;

    if ((bb->status & BAIO_STATUS_ACTIVE) == 0) return(-1);

    b = &bb->writeBuffer;
    dsize = baioSizeOfContinuousFreeSpaceForWrite(b);
    if (dsize <= 0) {
        dsize = 0;
        if (b->b == NULL) baioGetSpaceForWrite(bb, &bb->writeBuffer, 1);
    }
    va_copy(arg_ptr_copy, arg_ptr);
    n = vsnprintf(b->b + b->j, dsize, fmt, arg_ptr_copy);
#if _WIN32
    while (n < 0) {
        dsize = dsize * 2 + 1024;
        r = baioGetSpaceForWrite(bb, &bb->writeBuffer, dsize+1);
        if (r < 0) return(-1);
        n = vsnprintf(b->b + b->j, dsize, fmt, arg_ptr_copy);
    }
#endif
    InternalCheck(n >= 0);
    va_copy_end(arg_ptr_copy);
    if (n >= dsize) {
        r = baioGetSpaceForWrite(bb, &bb->writeBuffer, n+1);
        if (r < 0) return(-1);
        va_copy(arg_ptr_copy, arg_ptr);
        n = vsnprintf(b->b + b->j, n+1, fmt, arg_ptr_copy);
        InternalCheck(n>=0);
        va_copy_end(arg_ptr_copy);
    }

    b->j += n;
    return(n);
}

int baioPrintfToBuffer(struct baio *bb, char *fmt, ...) {
    int             res;
    va_list         arg_ptr;

    va_start(arg_ptr, fmt);
    res = baioVprintfToBuffer(bb, fmt, arg_ptr);
    va_end(arg_ptr);
    return(res);
}

//////////////////////////////////////////////////////////

static void baioNormalizeBufferBeforeRead(struct baioBuffer *b, struct baio *bb) {
    int delta; 

    if (b->i == b->j) {
        // we have previously processed all the buffer, reset indexes to start from the very beginning
        b->i = b->j = 0;
    } 
    if (b->b != NULL && b->size - b->j < bb->minFreeSizeBeforeRead + bb->minFreeSizeAtEndOfReadBuffer) {
	// not enough space for read
        // we rolled to the end of the buffer, move the trail to get enough of space
        baioBufferShift(bb, b);
    }
    if (b->b == NULL) {
	baioBufferCreate(b, bb->initialReadBufferSize);
    }
}

static void baioHandleSuccesfullRead(struct baio *bb, int n) {
    struct baioBuffer   *b;
    int                     sj;

    b = &bb->readBuffer;
    if (n == 0) {
        // end of file
        bb->status |= BAIO_STATUS_EOF_READ;
        // Hmm. maybe I shall call eof callback only when readbuffer is empty
        // Unfortunately this is not possible, because we can miss it (if read buffer is emptied out of baio).
        CALLBACK_CALL(bb->callBackOnEof, callBack(bb));
    } else if (n > 0) {
        sj = b->j;
        b->j += n;
        if (bb->minFreeSizeAtEndOfReadBuffer > 0) b->b[b->j] = 0;
        CALLBACK_CALL(bb->callBackOnRead, callBack(bb, sj, n));
    }
}

static int baioOnCanRead(struct baio *bb) {
    struct baioBuffer 		*b;
    int                     n;
    int                     minFreeSizeAtEndOfReadBuffer;

    b = &bb->readBuffer;
    minFreeSizeAtEndOfReadBuffer = bb->minFreeSizeAtEndOfReadBuffer;
    baioNormalizeBufferBeforeRead(b, bb);
    // printf("%s: calling read(%d, %p, %d)\n", currentLocalTime_st(), bb->fd, b->b+b->j, b->size-b->j-1-minFreeSizeAtEndOfReadBuffer);
    n = read(bb->rfd, b->b+b->j, b->size-b->j-1-minFreeSizeAtEndOfReadBuffer);
    if (baioDebugLevel > 20) {printf("read returned %d\n", n); fflush(stdout);}
    //if (n < 0) {printf("%s:%d: read returned %d: %s\n", __FILE__, __LINE__, n, JSVAR_STR_ERRNO()); fflush(stdout);}
    //if (n < 0 && (errno == EWOULDBLOCK || errno == EAGAIN || errno == EINTR)) n = 0;
    if (n < 0 ) return(baioCloseOnError(bb));
    baioHandleSuccesfullRead(bb, n);
    return(n);
}

static int baioWrite(struct baio *bb) {
    struct baioBuffer  *b;
    int                     n, err, len;

    b = &bb->writeBuffer;
    len = b->j - b->i;

    n = write(bb->wfd, b->b+b->i, len);
    if (baioDebugLevel > 20) {printf("write(,,%d) returned %d\n", b->j - b->i, n); fflush(stdout);}
    if (n < 0 && (errno == EWOULDBLOCK || errno == EAGAIN || errno == EINTR)) n = 0;
    return(n);
}

int baioOnCanWrite(struct baio *bb) {
    struct baioBuffer  *b;
    int                     n;
    int                     si;

    n = 0;
    b = &bb->writeBuffer;
    si = b->i;

    // only one write is allowed by select, go back to the main loop after a single write
    if (bb->wfd >= 0 && b->i < b->j) {
        n = baioWrite(bb);
        if (n < 0) return(baioCloseOnError(bb));
        b->i += n;
    }

    if (n != 0) CALLBACK_CALL(bb->callBackOnWrite, callBack(bb, si, n));
    if (b->i == b->j) {
        if (bb->status & BAIO_STATUS_PENDING_CLOSE) baioImmediateDeactivate(bb);
    }
    return(n);
}

/////////////////////


static int baioAddSelectFdToSet(int maxfd, int fd, fd_set *ss) {
    if (ss != NULL) {
        FD_SET(fd, ss);
        if (fd > maxfd) return(fd);
    }
    return(maxfd);
}

int baioSetSelectParams(int maxfd, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *t) {
    int             i, rfd, wfd;
    struct baio     *bb;
    unsigned        status;
    int             flagWaitingForReadFd;
    int             flagWaitingForWriteFd;

    for(i=0; i<baioTabMax; i++) {
        bb = baioTab[i];
        if (bb == NULL) goto nextfd;
        status = bb->status;
        // fd = bb->fd;
        rfd = bb->rfd;
        wfd = bb->wfd;
        flagWaitingForReadFd = flagWaitingForWriteFd = 0;
        // free zombies
        if (1 && bb->status == BAIO_STATUS_ZOMBIE) {
            baioFreeZombie(bb);
        } else if ((status & BAIO_STATUS_ACTIVE)) {
	    if (rfd >= 0) {
		// check for reading
		if (status & BAIO_BLOCKED_FOR_READ_IN_READ) {
		    flagWaitingForReadFd = 1;
		} else if (
		    (bb->ioDirections == BAIO_IO_DIRECTION_READ || bb->ioDirections == BAIO_IO_DIRECTION_RW)
		    && (status & BAIO_STATUS_EOF_READ) == 0
		    && (status & BAIO_STATUS_PENDING_CLOSE) == 0
		    && BAIO_READ_BUFFER_HAS_SPACE_FOR_READ(bb)
		    ) {
		    bb->status |= BAIO_BLOCKED_FOR_READ_IN_READ;
		    flagWaitingForReadFd = 1;
		}
	    }
	    if (wfd >= 0) {
		// check for writing
		if (status & BAIO_BLOCKED_FOR_WRITE_IN_WRITE) {
		    flagWaitingForWriteFd = 1;
		} else if (
		    (bb->ioDirections == BAIO_IO_DIRECTION_WRITE || bb->ioDirections == BAIO_IO_DIRECTION_RW)
		    // previously there was the !=, but it may be safer to use <
		    // && bb->writeBuffer.i != bb->writeBuffer.ij
		    && bb->writeBuffer.i < bb->writeBuffer.j
		    ) {
		    bb->status |= BAIO_BLOCKED_FOR_WRITE_IN_WRITE;
		    flagWaitingForWriteFd = 1;
		}
	    }
            if (flagWaitingForReadFd) {
                maxfd = baioAddSelectFdToSet(maxfd, rfd, readfds);
                maxfd = baioAddSelectFdToSet(maxfd, rfd, exceptfds);
            }
            if (flagWaitingForWriteFd) {
                maxfd = baioAddSelectFdToSet(maxfd, wfd, writefds);
                maxfd = baioAddSelectFdToSet(maxfd, wfd, exceptfds);
            }
            if (baioDebugLevel > 20) {
		printf("%s: Set      %d: fds %d, %d: status: %6x -> %6x; rwe : %d%d%d%d\n", PPREFIX(), i, rfd, wfd, status, bb->status, (readfds != NULL&&FD_ISSET(rfd, readfds)), (writefds != NULL && FD_ISSET(wfd, writefds)), (exceptfds != NULL && FD_ISSET(rfd, exceptfds)), (exceptfds != NULL && FD_ISSET(wfd, exceptfds))); fflush(stdout);
	    }
        }
    nextfd:;
    }
    return(maxfd+1);
}

int baioOnSelectEvent(int maxfd, fd_set *readfds, fd_set *writefds, fd_set *exceptfds) {
    int             i, rfd, wfd, max, res;
    unsigned        status;
    struct baio     *bb;

    res = 0;
    // loop until current baioTabMax, actions in the loop may add new descriptors, but those
    // were not waited at this moment. Do not check them
    max = baioTabMax;
    for(i=0; i<max; i++) {
        bb = baioTab[i];
        if (bb == NULL) goto nextfd;
        rfd = bb->rfd;
        wfd = bb->wfd;
        if (0 && bb->status == BAIO_STATUS_ZOMBIE) {
            baioFreeZombie(bb);
        } else {
	    if (rfd >= 0 && rfd < maxfd) {
		status = bb->status;
		bb->status &= ~(BAIO_STATUSES_CLEARED_PER_TICK);
		if (baioDebugLevel > 20) {
		    printf("%s: Event on %d: rfd %d: status: %6x    %6s; rwe: %d%d%d\n", PPREFIX(), i, rfd, status, "", (readfds != NULL&&FD_ISSET(rfd, readfds)), (writefds != NULL && FD_ISSET(rfd, writefds)), (exceptfds != NULL && FD_ISSET(rfd, exceptfds))); fflush(stdout);
		}
		if (exceptfds != NULL && FD_ISSET(rfd, exceptfds)) {
		    res ++;
		    baioImmediateDeactivate(bb);
		    goto nextfd;
		}
		if (readfds == NULL || FD_ISSET(rfd, readfds) == 0) {
		    // no activity on this fd, status remains unchanged
		    bb->status = status;
		} else {
		    res ++;
		    bb->lastActivityTime = time(NULL);
		    if (status & BAIO_BLOCKED_FOR_READ_IN_READ) baioOnCanRead(bb);
		}
	    }
	    if (wfd >= 0 && wfd < maxfd) {
		status = bb->status;
		bb->status &= ~(BAIO_STATUSES_CLEARED_PER_TICK);
		if (baioDebugLevel > 20) {
		    printf("%s: Event on %d: wfd %d: status: %6x    %6s; rwe: %d%d%d\n", PPREFIX(), i, wfd, status, "", (readfds != NULL&&FD_ISSET(wfd, readfds)), (writefds != NULL && FD_ISSET(wfd, writefds)), (exceptfds != NULL && FD_ISSET(wfd, exceptfds))); fflush(stdout);
		}
		// first check for ssl_pending
		if (exceptfds != NULL && FD_ISSET(wfd, exceptfds)) {
		    res ++;
		    baioImmediateDeactivate(bb);
		    goto nextfd;
		}
		if (writefds == NULL || FD_ISSET(wfd, writefds) == 0) {
		    // no activity on this fd, status remains unchanged
		    bb->status = status;
		} else {
		    res ++;
		    bb->lastActivityTime = time(NULL);
		    if (status & BAIO_BLOCKED_FOR_WRITE_IN_WRITE) baioOnCanWrite(bb);
		}
	    }
	}
        // if (jsVarDebugLevel > 40) baioBufferDump(&bb->writeBuffer);
    nextfd:;
    }
    return(res);
}

//////////////////////////////////////////////////////////

int baioSelect(int maxfd, fd_set *r, fd_set *w, fd_set *e, struct timeval *t) {
    int             res;
    res = select(maxfd, r, w, e, t);
    return(res);
}

int baioPoll2(int timeOutUsec, int (*addUserFds)(int maxfd, fd_set *r, fd_set *w, fd_set *e), void (*processUserFds)(int maxfd, fd_set *r, fd_set *w, fd_set *e)) {
    static fd_set          *r = NULL;
    static fd_set          *w = NULL;
    static fd_set          *e = NULL;
    void					*p;
    struct timeval  		t;
    int             		maxfd, res;

    if (r == NULL) {
	ALLOCC(p, SIZEOF_FD_SETSIZE, char); 
	r = (fd_set *) p;
	ALLOCC(p, SIZEOF_FD_SETSIZE, char); 
	w = (fd_set *) p;
	ALLOCC(p, SIZEOF_FD_SETSIZE, char); 
	e = (fd_set *) p;
    }

    EXTENDED_FD_ZERO(r);
    EXTENDED_FD_ZERO(w);
    EXTENDED_FD_ZERO(e);

    t.tv_sec = timeOutUsec / 1000000LL;
    t.tv_usec = timeOutUsec % 1000000LL; 
    maxfd = baioSetSelectParams(-1, r, w, e, &t);
    if (addUserFds != NULL) maxfd = addUserFds(maxfd, r, w, e);
    // printf("calling select, maxfd == %d\n", maxfd);fflush(stdout);
    res = baioSelect(maxfd, r, w, e, &t);
    setCurrentTime();
    if (res < 0) {
        if (errno != EINTR) {
            printf("%s: ERROR: select returned %d and errno == %d (%s)!\n", PPREFIX(), res, errno, strerror(errno));
        }
        EXTENDED_FD_ZERO(r); EXTENDED_FD_ZERO(w); EXTENDED_FD_ZERO(e); 
    }
    // printf("returning from select, maxfd == %d\n", maxfd);fflush(stdout);
    if (processUserFds != NULL) processUserFds(maxfd, r, w, e);
    baioOnSelectEvent(maxfd, r, w, e);
    return(maxfd);
}

int baioPoll(int timeOutUsec) {
    int res;
    res =  baioPoll2(timeOutUsec, NULL, NULL);
    return(res);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
// baio for regular file

struct baio *baioNewFile(char *path, int ioDirection, int additionalSpaceToAllocate) {
    struct baio     *bb;
    int             r, fd;
    unsigned        flags;

    if (path == NULL) return(NULL);

    if (ioDirection == BAIO_IO_DIRECTION_READ) flags = (O_RDONLY);
    else if (ioDirection == BAIO_IO_DIRECTION_WRITE) flags = (O_WRONLY | O_CREAT);
    else if (ioDirection == BAIO_IO_DIRECTION_RW) flags = (O_RDWR | O_CREAT);
    else {
        printf("%s: Invalid ioDirection\n", PPREFIX());
        return(NULL);
    }

    // printf("%s: Opening file %s:%o\n", JSVAR_PPREFIX(), path, flags);

    fd = open(path, flags, 00644);
    if (fd < 0) {
        printf("%s: Error: Can't open file %s (%s).\n", PPREFIX(), path, STR_ERRNO());
	return(NULL);
    }

    r = setFileNonBlocking(fd);
    if (r < 0) {
        printf("%s: Error: Can't set file descriptor to non-blocking state, closing it.\n", PPREFIX());
        close(fd);
        return(NULL);
    }

    bb = baioNewBasic(BAIO_TYPE_FILE, ioDirection, additionalSpaceToAllocate);
    bb->rfd = bb->wfd = fd;
    return(bb);
}

//////////////////////////////////////////////////////////
// baio for serial port

#if 0 // temporarily out, I will try to connected everything through pipes/sockets
struct baio *baioNewSerial(char *path, int baudrate, int ioDirection, int additionalSpaceToAllocate) {
    struct baio     *bb;
    int             r, fd;
    unsigned        flags;

    if (path == NULL) return(NULL);

    if (ioDirection == BAIO_IO_DIRECTION_READ) flags = (O_RDONLY);
    else if (ioDirection == BAIO_IO_DIRECTION_WRITE) flags = (O_WRONLY);
    else if (ioDirection == BAIO_IO_DIRECTION_RW) flags = (O_RDWR);
    else {
        printf("%s: Invalid ioDirection\n", PPREFIX());
        return(NULL);
    }

    // printf("%s: Opening file %s:%o\n", JSVAR_PPREFIX(), path, flags);

    fd = open(path, flags);
    if (fd < 0) {
        printf("%s: Error: Can't open serial %s (%s).\n", PPREFIX(), path, STR_ERRNO());
	return(NULL);
    }

    initSerialPort(fd, baudrate);
    r = setFileNonBlocking(fd);
    if (r < 0) {
        printf("%s: Error: Can't set serial file descriptor to non-blocking state, closing it.\n", PPREFIX());
        close(fd);
        return(NULL);
    }

    bb = baioNewBasic(BAIO_TYPE_SERIAL, ioDirection, additionalSpaceToAllocate);
    bb->fd = fd;
    return(bb);
}
#endif

//////////////////////////////////////////////////////////
// baio for UDP

#if 0 // temporarily out
struct baio *baioNewUDP(char *ip, int port, int ioDirection, int additionalSpaceToAllocate) {
    struct baio     *bb;
    int             r, fd;
    unsigned        flags;

    if (ip == NULL) return(NULL);

    if (ioDirection == BAIO_IO_DIRECTION_READ) flags = (O_RDONLY);
    else if (ioDirection == BAIO_IO_DIRECTION_WRITE) flags = (O_WRONLY);
    else if (ioDirection == BAIO_IO_DIRECTION_RW) flags = (O_RDWR);
    else {
        printf("%s: Invalid ioDirection\n", PPREFIX());
        return(NULL);
    }

    printf("%s: Connecting to UDP server: %s:%d\n", PPREFIX(), ip, port);

    fd = udp_connect(ip, port);
    if (fd < 0) {
        printf("%s: Error: Can't open UDP connection %s:%d (%s).\n", PPREFIX(), ip, port, STR_ERRNO());
	return(NULL);
    }

    r = setFileNonBlocking(fd);
    if (r < 0) {
        printf("%s: Error: Can't set UDP file descriptor to non-blocking state, closing it.\n", PPREFIX());
        close(fd);
        return(NULL);
    }

    bb = baioNewBasic(BAIO_TYPE_UDP, ioDirection, additionalSpaceToAllocate);
    bb->fd = fd;
    return(bb);
}
#endif


/////////////////////////////////////////////////////////////
// pipe

static void closeAllFdsFrom(int fd0) {
    int maxd, i;

    maxd = sysconf(_SC_OPEN_MAX);
    if (maxd > 1024) maxd = 1024;
    for(i=fd0; i<maxd; i++) close(i);
}

static int createPipesForPopens(int *in_fd, int *out_fd, int *pin, int *pout) {

    if (out_fd != NULL) {
        if (pipe(pin) != 0) {
            printf("%s: %s:%d: Can't create output pipe\n", PPREFIX(), __FILE__, __LINE__);
            return(-1);
        }
        *out_fd = pin[1];
        // printf("pipe pin: %p %p: %d %d\n", pin, pin+1, pin[0], pin[1]);
    }
    if (in_fd != NULL) {
        if (pipe(pout) != 0) {
            printf("%s: %s:%d: Can't create input pipe\n", PPREFIX(), __FILE__, __LINE__);
            if (out_fd != NULL) {
                close(pin[0]);
                close(pin[1]);
            }
            return(-1);
        }
        *in_fd = pout[0];
        // printf("pipe pout: %p %p: %d %d\n", pout, pout+1, pout[0], pout[1]);
    }
    return(0);
}

static void closePopenPipes(int *in_fd, int *out_fd, int pin[2], int pout[2]) {
    if (out_fd != NULL) {
        close(pin[0]);
        close(pin[1]);
    }
    if (in_fd != NULL) {
        close(pout[0]);
        close(pout[1]);
    }
}

pid_t popen2(char *command, int *in_fd, int *out_fd, int useBashFlag) {
    int                 pin[2], pout[2];
    pid_t               pid;
    int                 r, md;
    char                ccc[TMP_STRING_SIZE+10];

    if (createPipesForPopens(in_fd, out_fd, pin, pout) == -1) return(-1);

    pid = fork();

    if (pid < 0) {
        printf("%s: fork failed in popen2: %s\n", PPREFIX(), strerror(errno));
        closePopenPipes(in_fd, out_fd, pin, pout);
        return pid;
    }

    if (pid == 0) {
	// new task 
        if (out_fd != NULL) {
            close(pin[1]);
            dup2(pin[0], 0);
            close(pin[0]);
        } else {
            // do not inherit stdin, if no pipe is defined, close it.
            close(0);
        }

        // we do not want to loose completely stderr of the task. redirect it to a common file
        md = open("currentsubmsgs.txt", O_WRONLY | O_CREAT | O_APPEND, 0644);
        dup2(md, 2);
        close(md);

        if (in_fd != NULL) {
            close(pout[0]);
            dup2(pout[1], 1);
            close(pout[1]);
        } else {
            // if there is no pipe for stdout, join stderr.
            dup2(2, 1);
        }

        // close all remaining fds. This is important because otherwise files and pipes may remain open
        // until the new process terminates.
        closeAllFdsFrom(3);

        // Exec is better, because otherwise this process may be unkillable (we would kill the shell not the process itself).
        if (useBashFlag) {
            execlp("bash", "bash", "-c", command, NULL);
        } else {
            r = snprintf(ccc, sizeof(ccc), "exec %s", command);
	    if (r <= 0 || r >= sizeof(ccc)) {
		printf("%s: Command to execute too long: %s\n", PPREFIX(), command);
	    } else {
		execlp("sh", "sh", "-c", ccc, NULL);
	    }
        }
        // execlp(command, command, NULL);
        fprintf(stderr, "%s: Exec failed in popen2: %s\n", PPREFIX(), strerror(errno));
        exit(1);
    }

    if (in_fd != NULL) {
        close(pout[1]);
    }
    if (out_fd != NULL) {
        close(pin[0]);
    }

    return pid;
}

struct baio *baioNewPipedCommand(char *command, int ioDirection, int useBashFlag, int additionalSpaceToAllocate) {
    struct baio     *bb;
    int             r, rfd, wfd;
    char            *iod;

    if (command == NULL) return(NULL);

    rfd = wfd = -1;
    if (ioDirection == BAIO_IO_DIRECTION_READ) {
        r = popen2(command, &rfd, NULL, useBashFlag);
    } else if (ioDirection == BAIO_IO_DIRECTION_WRITE) {
        r = popen2(command, NULL, &wfd, useBashFlag);
    } else if (ioDirection == BAIO_IO_DIRECTION_RW) {
        r = popen2(command, &rfd, &wfd, useBashFlag);
    } else {
	printf("%s: Internal error: wrong baio ioDirection %d.\n", PPREFIX(), ioDirection);
	r = -1;
    }
    if (r < 0) return(NULL);
    if (rfd < 0 && wfd<0) return(NULL);
    setFileNonBlocking(rfd);
    setFileNonBlocking(wfd);
    bb = baioNewBasic(BAIO_TYPE_PIPED_COMMAND, ioDirection, additionalSpaceToAllocate);
    bb->rfd = rfd;
    bb->wfd = wfd;
    return(bb);
}

struct baio *baioNewNamedPipes(char *readPipePath, char *writePipePath, int additionalSpaceToAllocate) {
    struct baio     *bb;
    int             r, rfd, wfd;
    char            *iod;

    if (readPipePath == NULL && writePipePath == NULL) return(NULL);

    rfd = wfd = -1;
    if (readPipePath != NULL) {
	rfd = open(readPipePath, O_RDWR);
	if (rfd < 0) {
	    printf("%s: Error: Can't open read pipe %s (%s).\n", PPREFIX(), readPipePath, STR_ERRNO());
	    return(NULL);
	}
	setFileNonBlocking(rfd);
    }
    if (writePipePath != NULL) {
	wfd = open(writePipePath, O_RDWR);
	if (wfd < 0) {
	    printf("%s: Error: Can't open write pipe %s (%s).\n", PPREFIX(), writePipePath, STR_ERRNO());
	    if (rfd >= 0) close(rfd);
	    return(NULL);
	}
	setFileNonBlocking(wfd);
    }

    bb = baioNewBasic(BAIO_TYPE_NAMED_PIPES, BAIO_IO_DIRECTION_RW, additionalSpaceToAllocate);
    bb->rfd = rfd;
    bb->wfd = wfd;
    return(bb);
}
