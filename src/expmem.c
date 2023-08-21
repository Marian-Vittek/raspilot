///////////////////////////////////////////////////////////////////////////////////////////////////
// A memory allocator using lists of exponential sizes
// Shall be much faster than standard malloc

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>

#ifdef __APPLE__
#else
#include <malloc.h>
#endif
#include "expmem.h"


#define MAX_EXP_MEMORY_BLOCK_LOG 		31	/* we will not memorize blocks of size larger than 2^30 == 1GB */

#define SINGLE_THREAD_CHECK_ENTRY() {					\
	if (singleThreadEnteredFlag) {					\
	    printf("\t: Error: %s:%d: multiple threads are not supported!\n", __FILE__, __LINE__); \
	}								\
	singleThreadEnteredFlag = 1;					\
    }

#define SINGLE_THREAD_CHECK_EXIT(res) {					\
	if (singleThreadEnteredFlag == 0) {				\
	    printf("\t: Error: %s:%d: multiple threads are not supported!\n", __FILE__, __LINE__); \
	}								\
	singleThreadEnteredFlag = 0;					\
	return(res);							\
    }

typedef struct expmemFreeChunkHeader {
    union {
	struct expmemFreeChunkHeader *nextfree;
    } u;
} S_expmemFreeChunkHeader;


typedef struct expmemAllocatedChunkHeader {
    union {
	long long int		 freeInfo;
	char				 alignment[EXPMEM_REQUIRED_ALIGNMENT];
    } u;
} S_expmemAllocatedChunkHeader;

typedef struct expmemChunkHeader {
    union {
	long long int		 			freeInfo;		// allocated block
	struct expmemChunkHeader 		*nextfree;		// free block
	char				 			alignment[EXPMEM_REQUIRED_ALIGNMENT];
    } u;
} S_expmemChunkHeader;


/* ************************************************************************ */

static unsigned s_expmemPowers[MAX_EXP_MEMORY_BLOCK_LOG] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072, 262144, 524288, 1048576, 2097152, 4194304, 8388608, 16777216, 33554432, 67108864, 134217728, 268435456, 536870912, 1073741824}; 


static S_expmemChunkHeader 	*s_expmemChunks[MAX_EXP_MEMORY_BLOCK_LOG] = {NULL};

// Chunks of small sizes are allocted by "cutting off" from one large chunk
// Shall give better cache allocation for very small pieces of memory
static S_expmemChunkHeader 	*s_expmemDirectListChunks[EXPMEM_MAX_SIZE_FOR_DIRECT_LISTS] = {NULL};
static char 					*s_expmemDirectListPool;
static int 						s_expmemDirectListPoolRemainingSize;

// for debugging purposes
static long long				maxFreeingInfo = MAX_CHUNK_TO_KEEP_ALLOCATED;
static int						singleThreadEnteredFlag = 0;

/* ************************************************************************ */


static int expmemLog(int n) {
    int pos = 0;
    if (n >= 1<<16) { n >>= 16; pos += 16; }
    if (n >= 1<< 8) { n >>=  8; pos +=  8; }
    if (n >= 1<< 4) { n >>=  4; pos +=  4; }
    if (n >= 1<< 2) { n >>=  2; pos +=  2; }
    if (n >= 1<< 1) {           pos +=  1; }
    return pos;
}

int expmemNextRecommendedBufferSize(int previousSize) {
    int ns;
    if (previousSize <= sizeof(S_expmemChunkHeader)) {
	return(2 * sizeof(S_expmemChunkHeader));
    }
    ns = (1 << (expmemLog(previousSize)+2));
    return(ns - sizeof(S_expmemChunkHeader));
}


#if 0
static void expmemDump(FILE *ff) {
    unsigned 			i;
    S_expmemChunkHeader	*s;
    for(i=0; i<MAX_EXP_MEMORY_BLOCK_LOG; i++) {
	fprintf(ff, "%2d [%6u]: ", i, s_expmemPowers[i]);
	for(s=s_expmemChunks[i]; s!=NULL; s=s->nextfree) {
	    fprintf(ff, "%p,", s);
	}
	fprintf(ff, "\n");
    }
    fflush(ff);
}
#endif

static void expmemDumpCheckDoubleFree(struct expmemChunkHeader *elem) {
#if 0
    S_expmemChunkHeader	*s;
    for(s=elem; s!=NULL; s=s->u.nextfree) {
	if (s->u.nextfree == elem || s->u.nextfree == s) {
	    printf("\t: %s:%d double freeing in lists. Dumping core.\n",  __FILE__, __LINE__);
	    if (fork() == 0) abort();
	}
    }
#endif
}


static void expmemInsertToList(int i, struct expmemChunkHeader *elem) {
    elem->u.nextfree = s_expmemChunks[i];
    s_expmemChunks[i] = elem;
    expmemDumpCheckDoubleFree(s_expmemChunks[i]);
}

static struct expmemChunkHeader *expmemRemoveFromList(int i) {
    struct expmemChunkHeader *nf;

    expmemDumpCheckDoubleFree(s_expmemChunks[i]);
    nf = s_expmemChunks[i];
    if (nf != NULL ) s_expmemChunks[i] = nf->u.nextfree;
    return(nf);
}


//& the freeingInfo is the number to be sent to expmem0Free when freeing this block
static void *expmemInternalMalloc(int n, int *freeingInfo) {
    int		 				allocIndex, freeInfo;
    unsigned				i, nn;
    S_expmemChunkHeader		*space;
    char					*ss;

    if (n < sizeof(S_expmemChunkHeader)) n = sizeof(S_expmemChunkHeader);

    if (n < EXPMEM_MAX_SIZE_FOR_DIRECT_LISTS) {
	freeInfo = -n;
	if (s_expmemDirectListChunks[n] != NULL) {
	    space = s_expmemDirectListChunks[n];
	    s_expmemDirectListChunks[n] = s_expmemDirectListChunks[n]->u.nextfree;
	} else {
	    if (n > s_expmemDirectListPoolRemainingSize) {
		s_expmemDirectListPoolRemainingSize = DIRECT_LIST_ALLOCATIONS_POOL_SIZE;
		s_expmemDirectListPool = malloc(s_expmemDirectListPoolRemainingSize);
	    }
	    space = (S_expmemChunkHeader *) s_expmemDirectListPool;
	    if (space == NULL) goto fini;
	    // allign it for processor requiring allignement
	    nn = (n + (EXPMEM_REQUIRED_ALIGNMENT - 1)) & ~(EXPMEM_REQUIRED_ALIGNMENT - 1);
	    s_expmemDirectListPool += nn;
	    s_expmemDirectListPoolRemainingSize -= nn;
	}
	goto fini;
    }
    if (n > MAX_CHUNK_TO_KEEP_ALLOCATED) {
	space = (S_expmemChunkHeader *) malloc(n);
	freeInfo = n;
    } else {
	allocIndex = expmemLog(n);
	if (n != s_expmemPowers[allocIndex]) allocIndex++;
	freeInfo = allocIndex;
	space = s_expmemChunks[allocIndex];
	// allocSize = s_expmemPowers[allocIndex];
	if (space != NULL) {
	    s_expmemChunks[allocIndex] = space->u.nextfree;
	} else {
	    i = allocIndex;
	    assert(i>=0 && i <= MAX_CHUNK_LOG);
	    if (s_expmemPowers[i+1] <= MAX_CHUNK_TO_KEEP_ALLOCATED) {
		do {
		    i++;
		    space = expmemRemoveFromList(i);
		} while (space == NULL && s_expmemPowers[i+1] <= MAX_CHUNK_TO_KEEP_ALLOCATED);
	    }
	    assert(i>=0 && i <= MAX_CHUNK_LOG);
	    if (space == NULL) {
		i ++;
		// printf("allocating %d bytes\n", s_expmemPowers[i]);
		space = (struct expmemChunkHeader *) malloc(s_expmemPowers[i]);
		if (space == NULL) goto fini;
	    }
	    for(i--; i>=allocIndex; i--) {
		assert(i>=0 && i <= MAX_CHUNK_LOG);
		expmemInsertToList(i, space);
		ss = (char*)space;
		space = (struct expmemChunkHeader *) (ss + s_expmemPowers[i]);
	    }
	}
    }
fini:
    if (space == NULL) {
	// printf("Out of memory\n"); exit(1);
    }
    if (freeInfo > maxFreeingInfo) maxFreeingInfo = freeInfo;
    *freeingInfo = freeInfo;
    return((void*)space);
}

static void expmemInternalFree(void *p, int freeingInfo) {
    S_expmemChunkHeader *space;
    if (freeingInfo < 0) {
	if (-freeingInfo >= EXPMEM_MAX_SIZE_FOR_DIRECT_LISTS) {
	    printf("\t: %s:%d possible double freeing in lists.\n",  __FILE__, __LINE__);
#if EDEBUG
	    assert(0);
#endif
	} else {
	    int n = -freeingInfo;
	    space = (S_expmemChunkHeader *) p;
	    space->u.nextfree = s_expmemDirectListChunks[n];
	    s_expmemDirectListChunks[n] = space;
	}
    } else if (freeingInfo > MAX_CHUNK_TO_KEEP_ALLOCATED) {
	// you have probably not allocated 
	if (freeingInfo > maxFreeingInfo) {
	    printf("\t: %s:%d possible double freeing with malloc.\n",  __FILE__, __LINE__);
#if EDEBUG
	    assert(0);
#endif
	} else {
	    free(p);
	}
    } else {
	if (freeingInfo >= MAX_EXP_MEMORY_BLOCK_LOG) {
	    printf("\t: %s:%d possible double freeing with explist.\n",  __FILE__, __LINE__);
#if EDEBUG
	    assert(0);
#endif
	} else {
	    space = (S_expmemChunkHeader *) p;
	    space->u.nextfree = s_expmemChunks[freeingInfo];
	    s_expmemChunks[freeingInfo] = space;
	    expmemDumpCheckDoubleFree(s_expmemChunks[freeingInfo]);
	}
    }
}

static void *expmemInternalRealloc(void *p, int *freeingInfo, unsigned n) {
    int			 			freeInfo, oldsize, allocIndex;
    S_expmemChunkHeader 	*res, *space;

    if (p==NULL) return(expmemInternalMalloc(n, freeingInfo));
    if (n==0) {
	expmemInternalFree(p, *freeingInfo);
	return(NULL);
    }
    freeInfo = *freeingInfo;
    // calculate old size and check trivial case when no reallocation is needed.
    if (freeInfo < 0) {
	oldsize = -freeInfo;
	if (oldsize == n) return(p);
    } else if (freeInfo > MAX_CHUNK_TO_KEEP_ALLOCATED) {
	oldsize = freeInfo;
	if (oldsize == n) return(p);
    } else {
	oldsize =  s_expmemPowers[freeInfo];
	allocIndex = expmemLog(n);
	if (n != s_expmemPowers[allocIndex]) allocIndex++;
	if (freeInfo == allocIndex) return(p);
    }
    if (0 && oldsize >= n && oldsize <= MAX_CHUNK_TO_KEEP_ALLOCATED) {
	while (oldsize/2 > n) {
	    freeInfo --;
	    space = (S_expmemChunkHeader*)(((char*)p)+s_expmemPowers[freeInfo]);
	    space->u.nextfree = s_expmemChunks[freeInfo];
	    s_expmemChunks[freeInfo] = space;
	    oldsize = oldsize/2;
	}
	*freeingInfo = freeInfo;
	return(p);
    } else {
	res = expmemInternalMalloc(n, freeingInfo);
	if (res != NULL) {
	    if (oldsize < n) {
		memcpy(res, p, oldsize);
	    } else {
		memcpy(res, p, n);
	    }
	}
	expmemInternalFree(p, freeInfo);
	return(res);
    }
}


void *expmemMalloc(unsigned n) {
    int		 			n1, fi;
    S_expmemChunkHeader	*space;
    void 				*res;

    SINGLE_THREAD_CHECK_ENTRY();

    // if (n > 2024000) {printf("allocating %d bytes\n", n); fflush(stdout); *((int*)NULL) = 0;}
    // this is important, because otherwise we can not reallocate or free such a ptr
    if (n == 0) SINGLE_THREAD_CHECK_EXIT(NULL);

    n1 = n + sizeof(S_expmemChunkHeader);
    space = (S_expmemChunkHeader *) expmemInternalMalloc(n1, &fi);
#if 0
    if (space == NULL) {
	// a small hack only for BBO (does not compile under windows)
	extern int munlockall(void);
	printf("\n\n%s:%d: Warning: Insufficient RLIMIT_MEMLOCK. Allowing swap !!!\n\n", __FILE__, __LINE__);
	munlockall();
	space = (S_expmemChunkHeader *) expmemInternalMalloc(n1, &fi);
    }
#endif
    if (space == NULL) SINGLE_THREAD_CHECK_EXIT(NULL);
    space->u.freeInfo = fi;
    res = (void*)(space+1);
    //printf("Alloc %d: --> %p", n, res); expmemDump(stdout);
    SINGLE_THREAD_CHECK_EXIT(res);
}

int expmemFree(void *p) {
    S_expmemChunkHeader 	*space;
    int						fi;

    SINGLE_THREAD_CHECK_ENTRY();

    if (p == NULL) SINGLE_THREAD_CHECK_EXIT(0);
    space = ((S_expmemChunkHeader *)p)-1;
    fi = space->u.freeInfo;
    expmemInternalFree(space, fi);
    // printf("Free %d: ", fi); expmemDump(stdout);
    SINGLE_THREAD_CHECK_EXIT(0);
}

void *expmemRealloc(void *p, unsigned n) {
    S_expmemChunkHeader 	*space;
    int								n1, fi;

    if (p == NULL) return(expmemMalloc(n));

    if (n == 0) {
	expmemFree(p);
	return(NULL);
    }	

    SINGLE_THREAD_CHECK_ENTRY();

    n1 = n + sizeof(S_expmemChunkHeader);
    space = ((S_expmemChunkHeader *)p)-1;
    fi = space->u.freeInfo;
    space = expmemInternalRealloc(space, &fi, n1);
    if (space == NULL) SINGLE_THREAD_CHECK_EXIT(NULL);

    space->u.freeInfo = fi;
    // printf("Re-Alloc %d: ", n); expmemDump(stdout);
    SINGLE_THREAD_CHECK_EXIT((void*)(space+1));
}
