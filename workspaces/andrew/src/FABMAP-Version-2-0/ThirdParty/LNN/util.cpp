
/************************************************************************

Module: util.c (utility routines for memory allocation, etc.
Author: David Lowe (2006)

*************************************************************************/


#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "util.h"


/*----------------------- Error messages --------------------------------*/

/* Print message and quit. */
void FatalError(const char *msg)
{
    fprintf(stderr, "FATAL ERROR: %s\n",msg);
    abort();
}


/*-------------------- Pooled storage allocator ---------------------------*/

/* The following routines allow for the efficient allocation of storage in
     small chunks from a specified pool.  Rather than allowing each structure
     to be freed individually, an entire pool of storage is freed at once.
   This method has two advantages over just using malloc() and free().  First,
     it is far more efficient for allocating small objects, as there is
     no overhead for remembering all the information needed to free each
     object or consolidating fragmented memory.  Second, the decision about
     how long to keep an object is made at the time of allocation, and there
     is no need to track down all the objects to free them.
*/

/* We maintain memory alignment to word boundaries by requiring that all
   allocations be in multiples of the machine wordsize.
*/
#define WORDSIZE 4   /* Size of machine word in bytes.  Must be power of 2. */
#define BLOCKSIZE 2048    /* Minimum number of bytes requested at a time from
               the system.  Must be multiple of WORDSIZE. */


/* Return a pointer to a new storage pool.
*/
Pool CreatePool()
{
    Pool pool;

    pool = (PoolSt*)malloc(sizeof(struct PoolSt));
    pool->remaining = 0;
    pool->base = NULL;
    return pool;
}

/* Returns a pointer to a piece of new memory of the given size in bytes
   allocated from the pool.
*/
void *MallocPool(int size, Pool pool)
{
    char *m, *rloc;
    int blocksize;

    /* Round size up to a multiple of wordsize.  The following expression
       only works for WORDSIZE that is a power of 2, by masking last bits of
       incremented size to zero.
    */
    size = (size + WORDSIZE - 1) & ~(WORDSIZE - 1);

    /* Check whether a new block must be allocated.  Note that the first word
       of a block is reserved for a pointer to the previous block.
    */
    if (size > pool->remaining) {
    blocksize = (size + sizeof(void *) > BLOCKSIZE) ?
               size + sizeof(void *) : BLOCKSIZE;
    m = (char*) malloc(blocksize);
    if (! m)
        FatalError("Failed to allocate memory.");
    pool->remaining = blocksize - sizeof(void *);
    /* Fill first word of new block with pointer to previous block. */
    ((char **) m)[0] = pool->base;
    pool->base = m;
    pool->loc = m + sizeof(void *);
    }
    /* Allocate new storage. */
    rloc = pool->loc;
    pool->loc += size;
    pool->remaining -= size;
    return rloc;
}


/* Free all storage that was previously allocated to this pool.
*/
void FreePool(Pool pool)
{
    char *prev;

    while (pool->base != NULL) {
    prev = *((char **) pool->base);  /* Get pointer to prev block. */
    free(pool->base);
    pool->base = prev;
    }
    free(pool);
}
