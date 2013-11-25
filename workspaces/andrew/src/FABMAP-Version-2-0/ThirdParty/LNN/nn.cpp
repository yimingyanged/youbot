
/************************************************************************

Module: nn.c (approximate nearest-neighbor matching)
Author: David Lowe (2006)

nn.c:
  This module finds the nearest-neighbors of vectors in high dimensional
      spaces using a search of multiple randomized k-d trees.

  The following routines are the interface to this module:

  Index BuildIndex(unsigned char **vecs, int vcount, int veclen, int numTrees)
    This routine creates an Index data structure containing the k-d trees
    and other information used to find neighbors to the given set of vectors.
        vecs: array of pointers to the vectors to be indexed.  Each vector
              value is assumed to be unsigned char (so in range [0,255]).
        vcount: number of vectors in vecs.
        veclen: the length of each vector.
        numTrees: the number of randomized trees to build.

  Index BuildIndexFloat(float **fvecs, int vcount, int veclen, int numTrees)
    The same as BuildIndex, but vectors contain float values.

  FreeIndex(Index index)
    Frees all memory for the given index.

  FindNeighbors(int *result, int numNN, unsigned char *vec, Index index,
                int maxCheck)
    Find the numNN nearest neighbors to vec and store their indices in the
    "result" vector, which must have length numNN.  The returned indices
    refer to position in the original vecs used to create the index.
    Seach a maximum of maxCheck tree nodes for the result (this is what
    determines the amount of computation).

  FindNeighborsFloat(int *result, int numNN, float *vec, Index index)
    Same as FindNeighbors, but takes a vector with float values.

*************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "nn.h"


/*--------------------------- Constants -----------------------------*/

/* When creating random trees, the dimension on which to subdivide is
   selected at random from among the top RandDim dimensions with the
   highest variance.  A value of 5 works well.
*/
#define RandDim 5

/* To improve efficiency, only SampleMean random values are used to
   compute the mean and variance at each level when building a tree.
   A value of 100 seems to perform as well as using all values.
*/
const int SampleMean = 100;


/*--------------------- Internal Data Structures --------------------------*/

/* This is a node of the binary k-d tree.  All nodes that have 
   vec[divfeat] < divval are placed in the child1 subtree, else child2.
   A leaf node is indicated if both children are NULL.
*/
typedef struct TreeSt {
    int divfeat;    /* Index of the vector feature used for subdivision.
               If this is a leaf node (both children are NULL) then
               this holds vector index for this leaf. */
    float divval;   /* The value used for subdivision. */
    struct TreeSt *child1, *child2;  /* Child nodes. */
} *Tree;


/* This record represents a branch point when finding neighbors in
   the tree.  It contains a record of the minimum distance to the query
   point, as well as the node at which the search resumes.
*/
typedef struct BranchSt {
    Tree node;           /* Tree node at which search resumes */
    float mindistsq;     /* Minimum distance to query for all nodes below. */
} *Branch;


/* -------------------- Local function prototypes ------------------------ */

Index CreateIndex(unsigned char **vecs, float **fvecs, int vcount, int veclen,
          int numTrees);
void DivideTree(Tree *pTree, Index index, int first, int last);
void ChooseDivision(Tree node, Index index, int first, int last);
int SelectDiv(float *v, int vlen);
void Subdivide(Tree node, Index index, int first, int last);
void GetNeighbors(int *result, int numNN, unsigned char *vec, float *fvec,
          Index index, int maxCheck);
void SearchLevel(int *result, int numNN, unsigned char *vec, float *fvec,
         Index index, Tree node, float mindistsq, int maxCheck);
void HeapInsert(Index index, Tree node, int mindistsq);
Branch HeapMin(Index index);
void Heapify(Index index, int parent);
void CheckNeighbor(int *result, int numNN, int vindex, unsigned char *vec,
           float *fvec, Index index);
float IntDistSquared(unsigned char *v1, unsigned char *v2, int veclen);
float FloatDistSquared(float *v1, float *v2, int veclen);


#ifdef WIN32
    double drand48()
    {
        return (double)rand()/(double)(RAND_MAX+1);
    }
#endif

/*------------------------ Build k-d tree index ---------------------------*/


/* Build and return the k-d tree index used to find nearest neighbors to
   a set of vectors.  The vector values are of type unsigned char.
     vecs: array of pointers to the vectors to be indexed.
     vcount: number of vectors in vecs.
     veclen: the length of each vector.
     numTrees: the number of randomized trees to build.
*/
Index BuildIndex(unsigned char **vecs, int vcount, int veclen, int numTrees)
{
  return CreateIndex(vecs, NULL, vcount, veclen, numTrees);
}


/* This is the same as BuildIndex, but takes vectors of float values.
   While less efficient in memory use, this is more convenient for general
   purpose use, as with Matlab.
*/
Index BuildIndexFloat(float **fvecs, int vcount, int veclen, int numTrees)
{
  return CreateIndex(NULL, fvecs, vcount, veclen, numTrees);
}


/* Returns an index structure containing the k-d trees for these vectors.
   Only one of vecs or fvecs is used, and the other must be NULL.
*/
Index CreateIndex(unsigned char **vecs, float **fvecs, int vcount, int veclen,
          int numTrees)
{
    int i, j, rand, temp;
    Index index;
    Pool pool;

    pool = CreatePool();    /* All data for the index goes into this pool. */
    index = NEW(IndexSt, pool);
    index->pool = pool;
    index->numTrees = numTrees;
    index->vcount = vcount;
    index->veclen = veclen;
    index->vecs = vecs;
    index->fvecs = fvecs;
    index->trees = (Tree *) MallocPool(numTrees * sizeof(Tree), pool);
    index->maxHeap = 512;    /* Should be power of 2. */
    index->heap =    /* Heap uses 1-based indexing, so needs extra element. */
        (Branch) MallocPool((1 + index->maxHeap) * sizeof(struct BranchSt),
                pool);
    index->dsqlen = 0;
    index->checkID = -1000;

    /* Create a permutable array of indices to the input vectors. */
    index->vind = (int *) MallocPool(vcount * sizeof(int), pool);
    for (i = 0; i < vcount; i++)
      index->vind[i] = i;

    /* Construct the randomized trees. */
    for (i = 0; i < numTrees; i++) {

      /* Randomize the order of vectors to allow for unbiased sampling. */
      for (j = 0; j < vcount; j++) {
      rand = (int) (drand48() * vcount);  
      assert(rand >=0 && rand < vcount);
    temp = index->vind[j];
    index->vind[j] = index->vind[rand];
    index->vind[rand] = temp;
      }
      index->trees[i] = NULL;
      DivideTree(& index->trees[i], index, 0, vcount - 1);
    }
    return index;
}


/* Free all memory used to create this index.
*/
void FreeIndex(Index index)
{
  FreePool(index->pool);
}


/* Create a tree node that subdivides the list of vecs from vind[first]
   to vind[last].  The routine is called recursively on each sublist.
   Place a pointer to this new tree node in the location pTree. 
*/
void DivideTree(Tree *pTree, Index index, int first, int last)
{
    Tree node;

    node = NEW(TreeSt, index->pool);
    *pTree = node;

    /* If only one exemplar remains, then make this a leaf node. */
    if (first == last) {
      node->child1 = node->child2 = NULL;    /* Mark as leaf node. */
      node->divfeat = index->vind[first];    /* Store index of this vec. */
    } else {
      ChooseDivision(node, index, first, last);
      Subdivide(node, index, first, last);
    }
}


/* Choose which feature to use in order to subdivide this set of vectors.
   Make a random choice among those with the highest variance, and use
   its variance as the threshold value.
*/
void ChooseDivision(Tree node, Index index, int first, int last)
{
    int i, j, ind, end, count = 0;
    float dist, val;
    static int meanlen = 0;
    static float *mean, *var;

    /* Allocate scratch space for the mean and var vectors. */
    if (meanlen < index->veclen) {
      meanlen = index->veclen;
      mean = (float *) malloc(meanlen * sizeof(float));
      var = (float *) malloc(meanlen * sizeof(float));
    }
    for (i = 0; i < index->veclen; i++) {
      mean[i] = 0.0;
      var[i] = 0.0;
    }
    /* Compute mean values.  Only the first SampleMean values need to be
       sampled to get a good estimate.
    */
    end = MIN(first + SampleMean, last);
    for (j = first; j <= end; j++) {
      count++;
      ind = index->vind[j];
      for (i = 0; i < index->veclen; i++)
    mean[i] += (index->vecs == NULL) ? index->fvecs[ind][i] :
                                           (float) index->vecs[ind][i];
    }
    for (i = 0; i < index->veclen; i++)
      mean[i] /= count;

    /* Compute variances (no need to divide by count). */
    for (j = first; j <= end; j++) {
      ind = index->vind[j];
      for (i = 0; i < index->veclen; i++) {
    val = (index->vecs == NULL) ? index->fvecs[ind][i] :
                                  (float) index->vecs[ind][i];
    dist = val - mean[i];
    var[i] += dist * dist;
      }
    }
    /* Select one of the highest variance indices at random. */
    node->divfeat = SelectDiv(var, index->veclen);
    node->divval = mean[node->divfeat];
}
      

/* Select the top RandDim largest values from v and return the index of
   one of these selected at random.
*/
int SelectDiv(float *v, int vlen)
{
    int i, j, topind[RandDim], temp, rand, num = 0;
    
    /* Create a list of the indices of the top RandDim values. */
    for (i = 0; i < vlen; i++) {
      if (num < RandDim  ||  v[i] > v[topind[num-1]]) {
    /* Put this element at end of topind. */
    if (num < RandDim)
      topind[num++] = i;            /* Add to list. */
    else topind[num-1] = i;         /* Replace last element. */
    /* Bubble end value down to right location by repeated swapping. */
    j = num - 1;
    while (j > 0  &&  v[topind[j]] > v[topind[j-1]]) {
      temp = topind[j];
      topind[j] = topind[j-1];
      topind[j-1] = temp;
      j--;
    }
      }
    }
    /* Select a random integer in range [0,num-1], and return that index. */
    rand = (int) (drand48() * num);
    assert(rand >=0 && rand < num);
    return topind[rand];
}


/* Subdivide the list of exemplars using the feature and division
   value given in this node.  Call DivideTree recursively on each list.
*/
void Subdivide(Tree node, Index index, int first, int last)
{
    int i, j, ind, temp;
    float val;

    /* Move vector indices for left subtree to front of list. */
    i = first;
    j = last;
    while (i <= j) {
      ind = index->vind[i];
      val = (index->vecs == NULL) ? index->fvecs[ind][node->divfeat] :
                                    (float) index->vecs[ind][node->divfeat];
      if (val < node->divval) {
    i++;
      } else {
    /* Move to end of list by swapping vind i and j. */
    temp = index->vind[i];
    index->vind[i] = index->vind[j];
    index->vind[j] = temp;
    j--;
      }
    }
    /* If either list is empty, it means we have hit the unlikely case
       in which all remaining features are identical.  We move one
       vector to the empty list to avoid need for special case.
    */
    if (i == first)
      i++;
    if (i == last + 1)
      i--;

    DivideTree(& node->child1, index, first, i - 1);
    DivideTree(& node->child2, index, i, last);
}


/*----------------------- Nearest Neighbor Lookup ------------------------*/



/* Find set of numNN nearest neighbors to vec, and place their indices
   (location in original vector given to BuildIndex) in result.
*/
void FindNeighbors(int *result, int numNN, unsigned char *vec, Index index,
           int maxCheck)
{
    GetNeighbors(result, numNN, vec, NULL, index, maxCheck);
}


/* Same as Neighbors, but takes a floating point vector.
*/
void FindNeighborsFloat(int *result, int numNN, float *vec, Index index,
            int maxCheck)
{
    GetNeighbors(result, numNN, NULL, vec, index, maxCheck);
}


void GetNeighbors(int *result, int numNN, unsigned char *vec, float *fvec,
            Index index, int maxCheck)
{
    int i;
    Branch branch;

    index->ncount = 0;
    index->checkCount = 0;
    index->heapLen = 0;
    index->checkID -= 1;  /* Set a different unique ID for each search. */

    /* Make sure index->dsqs is long enough to hold numNN values. */
    if (numNN > index->dsqlen) {
      index->dsqs = (float *) MallocPool(numNN * sizeof(float), index->pool);
      index->dsqlen = numNN;
    }

    /* Search once through each tree down to root. */
    for (i = 0; i < index->numTrees; i++)
      SearchLevel(result, numNN, vec, fvec, index, index->trees[i], 0.0,
          maxCheck);

    /* Keep searching other branches from heap until finished. */
    while ((branch = HeapMin(index)) != NULL 
       && (index->checkCount < maxCheck  ||
           index->ncount < numNN))
      SearchLevel(result, numNN, vec, fvec, index, branch->node,
          branch->mindistsq, maxCheck);
    assert(index->ncount == numNN);
}


/* Search starting from a given node of the tree.  Based on any mismatches at
   higher levels, all exemplars below this level must have a distance of
   at least "mindistsq". 
*/
void SearchLevel(int *result, int numNN, unsigned char *vec, float *fvec,
         Index index, Tree node, float mindistsq, int maxCheck)
{
    float val, diff;
    Tree bestChild, otherChild;

    /* If this is a leaf node, then do check and return. */
    if (node->child1 == NULL  &&  node->child2 == NULL) {
    CheckNeighbor(result, numNN, node->divfeat, vec, fvec, index);
    return;
    }

    /* Which child branch should be taken first? */
    val = (vec == NULL) ? fvec[node->divfeat] : (float) vec[node->divfeat];
    diff = val - node->divval;
    bestChild = (diff < 0) ? node->child1 : node->child2;
    otherChild = (diff < 0) ? node->child2 : node->child1;

    /* Create a branch record for the branch not taken.  Add distance
       of this feature boundary (we don't attempt to correct for any
       use of this feature in a parent node, which is unlikely to
       happen and would have only a small effect).  Don't bother
       adding more branches to heap after halfway point, as cost of
       adding exceeds their value.
    */
    if (2 * index->checkCount < maxCheck  ||  index->ncount < numNN)
      HeapInsert(index, otherChild, (int) (mindistsq + diff * diff));

    /* Call recursively to search next level down. */
    SearchLevel(result, numNN, vec, fvec, index, bestChild, mindistsq,
        maxCheck);
}


/*------------------------- Priority Queue -----------------------------*/

/* The priority queue is implemented with a heap.  A heap is a complete
   (full) binary tree in which each parent is less than both of its
   children, but the order of the children is unspecified.
     Note that a heap uses 1-based indexing to allow for power-of-2
   location of parents and children.  We ignore element 0 of Heap array.
*/

/* Insert a new element in the heap, with values "node" and "mindistsq".
   We select the next empty leaf node, and then keep moving any larger
   parents down until the right location is found to store this element.
*/
void HeapInsert(Index index, Tree node, int mindistsq)
{
    int loc, par;

    /* If heap is full, then return without adding this element. */
    if (index->heapLen == index->maxHeap)
      return;

    loc = ++(index->heapLen);   /* Remember 1-based indexing. */

    /* Keep moving parents down until a place is found for this node. */
    par = loc / 2;                 /* Location of parent. */
    while (par > 0  &&  index->heap[par].mindistsq > mindistsq) {
      index->heap[loc] = index->heap[par];     /* Move parent down to loc. */
      loc = par;
      par = loc / 2;
    }
    /* Insert the element at the determined location. */
    index->heap[loc].node = node;
    index->heap[loc].mindistsq = (float) mindistsq;
}


/* Return the node from the heap with minimum value.  Reorganize
   to maintain the heap.
*/
Branch HeapMin(Index index)
{
    struct BranchSt temp;

    if (index->heapLen == 0)
      return NULL;

    /* Switch first node with last. */
    temp = index->heap[1];
    index->heap[1] = index->heap[index->heapLen];
    index->heap[index->heapLen] = temp;

    index->heapLen -= 1;
    Heapify(index, 1);      /* Move new node 1 to right position. */

    return & (index->heap[index->heapLen + 1]);  /* Return old last node. */
}


/* Take a heap rooted at position "parent" and enforce the heap critereon
   that a parent must be smaller than its children.
*/
void Heapify(Index index, int parent) 
{
    int left, right, minloc = parent;
    struct BranchSt temp;

    /* Check the left child */
    left = 2 * parent;
    if (left <= index->heapLen  &&
    index->heap[left].mindistsq < index->heap[parent].mindistsq)
      minloc = left;

    /* Check the right child */
    right = left + 1;
    if (right <= index->heapLen  && 
    index->heap[right].mindistsq < index->heap[minloc].mindistsq)
      minloc = right;

    /* If a child was smaller, than swap parent with it and Heapify. */
    if (minloc != parent) {
      temp = index->heap[parent];
      index->heap[parent] = index->heap[minloc];
      index->heap[minloc] = temp;
      Heapify(index, minloc);
    }
}


/*-------------------- Final check of each leaf node ----------------------*/

/* Measure distance from vec or fvec to the vector indicated by vindex
   and add it to the result list if appropriate.
*/
void CheckNeighbor(int *result, int numNN, int vindex, unsigned char *vec,
            float *fvec, Index index)
{
    int i, ncount, temp;
    float distsq, ftemp, *dsqs;

    index->checkCount += 1;

    /* Do not check same node more than once when searching multiple trees.
       Once a vector is checked, we set its location in index->vind to the
       current index->checkID.
    */
    if (index->vind[vindex] == index->checkID)
      return;
    index->vind[vindex] = index->checkID;

    distsq = (vec == NULL) ? 
                FloatDistSquared(fvec, index->fvecs[vindex], index->veclen) :
                IntDistSquared(vec, index->vecs[vindex], index->veclen);
    dsqs = index->dsqs;   /* Vector of distances to current results. */
    ncount = index->ncount;

    /* If result list is not full, then add this to end. */
    if (ncount < numNN) {
      result[ncount] = vindex;
      dsqs[ncount++] = distsq;
      index->ncount += 1;

    /* Replace last element of result with this, if it is closer. */
    } else if (distsq < index->dsqs[ncount-1]) {
      result[ncount-1] = vindex;
      dsqs[ncount-1] = distsq;

    /* Else return, as distance is larger than existing neighbors. */
    } else
      return;

    /* Bubble last element of results down to correct position. */
    i = ncount - 1;
    while (i >= 1  &&  dsqs[i] < dsqs[i-1]) {
      /* Swap results in i and i-1. */
      temp = result[i];
      result[i] = result[i-1];
      result[i-1] = temp;
      ftemp = dsqs[i];
      dsqs[i] = dsqs[i-1];
      dsqs[i-1] = ftemp;
      i--;
    }
}


/* Return the squared distance between two vectors of unsigned char
   type. This is highly optimized, with loop unrolling, as it is one
   of the most expensive inner loops of recognition.
*/
float IntDistSquared(unsigned char *v1, unsigned char *v2, int veclen)
{
    int diff, distsq = 0;
    int diff0, diff1, diff2, diff3;
    unsigned char *final, *finalgroup;

    final = v1 + veclen;
    finalgroup = final - 3;

    /* Process 4 pixels with each loop for efficiency. */
    while (v1 < finalgroup) {
      diff0 = v1[0] - v2[0];
      diff1 = v1[1] - v2[1];
      diff2 = v1[2] - v2[2];
      diff3 = v1[3] - v2[3];
      distsq += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
      v1 += 4;
      v2 += 4;
    }
    /* Process last 0-3 pixels.  Not needed for standard vector lengths. */
    while (v1 < final) {
      diff = *v1++ - *v2++;
      distsq += diff * diff;
    }
    return (float) distsq;
}


/* Same as IntDistSquared but for vectors of floats.
 */

float FloatDistSquared(float *v1, float *v2, int veclen)
{
    float diff, distsq = 0.0;
    float diff0, diff1, diff2, diff3;
    float *final, *finalgroup;

    final = v1 + veclen;
    finalgroup = final - 3;

    /* Process 4 pixels with each loop for efficiency. */
    while (v1 < finalgroup) {
      diff0 = v1[0] - v2[0];
      diff1 = v1[1] - v2[1];
      diff2 = v1[2] - v2[2];
      diff3 = v1[3] - v2[3];
      distsq += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
      v1 += 4;
      v2 += 4;
    }
    /* Process last 0-3 pixels.  Not needed for standard vector lengths. */
    while (v1 < final) {
      diff = *v1++ - *v2++;
      distsq += diff * diff;
    }
    return distsq;
}


