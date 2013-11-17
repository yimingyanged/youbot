
#include "util.h"

/*---------------------------- Structures -------------------------------*/

/* Contains the k-d trees and other information for indexing a set of points
   for nearest-neighbor matching.
*/
typedef struct IndexSt {
  int numTrees;       /* Number of randomized trees that are used. */
  int checkCount;     /* Number of neighbors checked so far in this lookup. */
  float searchDistSq; /* Distance cutoff for searching (not used yet). */
  int vcount;         /* Number of vectors stored in this index. */
  int veclen;         /* Length of each vector. */
  unsigned char **vecs; /* Array of vectors used as input. */
  float **fvecs;      /* Float vecs.  One of vecs and fvecs must be NULL. */
  int *vind;          /* Array of indices to vecs or fvecs.  When doing
                 lookup, this is used instead to mark checkID. */
  int checkID;        /* A unique ID for each lookup. */
  int ncount;         /* Number of neighbors so far in result. */
  float *dsqs;        /* Squared distances to current results. */
  int dsqlen;         /* Length of space allocated for dsqs. */
  struct TreeSt **trees;  /* Array of k-d trees used to find neighbors. */
  struct BranchSt *heap;  /* Heap of branches to explore. */
  int maxHeap;        /* Maximum size of heap. */
  int heapLen;        /* Current size of heap. */
  Pool pool;          /* Memory pool that holds all data for this index. */
} *Index;


/*-------------------- External function prototypes ---------------------*/

Index BuildIndex(unsigned char **vecs, int vcount, int veclen, int numTrees);
Index BuildIndexFloat(float **fvecs, int vcount, int veclen, int numTrees);
void FreeIndex(Index index);
void FindNeighbors(int *result, int numNN, unsigned char *vec, Index index,
           int maxCheck);
void FindNeighborsFloat(int *result, int numNN, float *vec, Index index,
            int maxCheck);
float IntDistSquared(unsigned char *v1, unsigned char *v2, int veclen);
float FloatDistSquared(float *v1, float *v2, int veclen);


