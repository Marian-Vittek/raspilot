///////////////////////////////////////////////////////////////////////////////////////////////////
// Header for a memory allocator using lists of exponential sizes
// Shall be much faster than standard malloc

/* alignment at  which returned pointers are rounded.  Intel does not
 * require alignment, however it is a good habit  */
#define EXPMEM_REQUIRED_ALIGNMENT		16

/* Only  chunks smaller than  MAX_CHUNK_TO_KEEP_ALLOCATED are  kept in
 * exponential free lists. Really big chunks are allocated directly by
 * malloc  */
#define MAX_CHUNK_LOG 				  	16   		/* log(MAX_CHUNK_TO_KEEP_ALLOCATED) */
#define MAX_CHUNK_TO_KEEP_ALLOCATED 	(1<<MAX_CHUNK_LOG) 		/* has to be a power of two! */


/* Very  small  chunks  (i.e.    chunks  smaller  than  the  following
 * constant) aren't  saved in  exponentially indexed lists.   They are
 * stored in lists per size (not log of size).  Computing of logarithm
 * of the size is saved and a  better cache usage may be expected at a
 * price of  some fragmentation.  
*/
#define EXPMEM_MAX_SIZE_FOR_DIRECT_LISTS	(1<<10)

/* Those very small allocated pieces are "cut off" from a chunk of the
 * following size (allocated directly by malloc) */
#define DIRECT_LIST_ALLOCATIONS_POOL_SIZE	(1<<16)



/* the standard provided functions */
extern void *expmemMalloc(unsigned n);
extern int   expmemFree(void *p);
extern void *expmemRealloc(void *p, unsigned n);

int expmemNextRecommendedBufferSize(int previousSize) ;
//extern void expmemDump(FILE *ff);

