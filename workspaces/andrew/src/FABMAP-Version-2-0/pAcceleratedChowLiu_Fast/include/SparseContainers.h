#ifndef ACC_CHOW_LIU_SPARSE_CONTAINERS_H
#define ACC_CHOW_LIU_SPARSE_CONTAINERS_H 1

#include "DataTypes.h"
#include <stxxl/vector>
#include <stxxl/sort>
#include <stxxl/scan>
#include <limits>
//#include <stxxl/priority_queue>
//#include <boost/pending/relaxed_heap.hpp>
//#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <queue>

typedef priority_queue<EdgeWithParams,std::vector<EdgeWithParams>,EdgeWithParams_Comparator_LargestFirst> PriorityHeap;    //STL priority_queue does not support changing the priority of an element already in the heap

struct Cooccurence
{
    NodeID nID1;
    NodeID nID2;
};

class Cooccurence_Comparator_SmallestFirst
{
public:
    bool operator () (const Cooccurence & a, const Cooccurence & b) const 
    {
        return a.nID1<b.nID1 || (a.nID1==b.nID1 && a.nID2 < b.nID2);
    }

    Cooccurence min_value() const  { return minval; }
    Cooccurence max_value() const  { return maxval; }

    Cooccurence_Comparator_SmallestFirst()
    {
        minval.nID1 = (std::numeric_limits<unsigned int>::min)();
        minval.nID2 = (std::numeric_limits<unsigned int>::min)();

        maxval.nID1 = (std::numeric_limits<unsigned int>::max)();
        maxval.nID2 = (std::numeric_limits<unsigned int>::max)();
    }
private:
    Cooccurence minval;
    Cooccurence maxval;
};



//Parameters of external memory Cooccurence Container
//For a discussion of parameters, see
//http://algo2.iti.uni-karlsruhe.de/dementiev/files/stxxl_tutorial.pdf
//Total internal memory consumption is NO_OF_PAGES*PAGE_SIZE*BLOCK_SIZE bytes
const unsigned int EMC_DEFAULT_PAGE_SIZE = 1;//4;
const unsigned int EMC_DEFAULT_NO_OF_PAGES = 1;//4;
const unsigned int EMC_DEFAULT_BLOCK_SIZE = 1*1024*1024;//2*1024*1024;

// internal memory for external sorting
const unsigned int EMC_INTERNAL_MEMORY_FOR_SORTING = 640*1024*1024;

//internal memory for for_each traversal, in blocks
const unsigned int  EMC_DEFAULT_NUM_BLOCKS_FOR_SCAN = 8;

typedef stxxl::VECTOR_GENERATOR<Cooccurence,EMC_DEFAULT_PAGE_SIZE,EMC_DEFAULT_NO_OF_PAGES,EMC_DEFAULT_BLOCK_SIZE>::result CooccurenceContainer;






//Comparator for STXXl priority queue.
/*
class CoOccurence_PQ_Comparator_SmallestFirst
{
public:
    bool operator () (const CoOccurence & a, const CoOccurence & b) const 
    {
        return a.nID1>b.nID1 || (a.nID1==b.nID1 && a.nID2 > b.nID2);
    }

    CoOccurence min_value() const  { return minval; }

    CoOccurence_Comparator_SmallestFirst()
        :minval((std::numeric_limits<unsigned int>::max)(),(std::numeric_limits<unsigned int>::max)())
    {}
private:
    CoOccurence minval;
};
*/

//typedef stxxl::PRIORITY_QUEUE_GENERATOR<CoOccurence,CoOccurence_Comparator_SmallestFirst> pqueue;

//We use a priority heap to store the egdes in order of weight, for input to Kruskal's algorithm

//We also use heaps when constructing the cooccurence lists
//Here we just want to stote (key,value) pairs in some sparse but efficient structure.
//And heap is convenient

//Update: turns out it's not. Boost heap structures are not efficient.
//Use sparse matrix (tree-based) instead
//typedef boost::numeric::ublas::mapped_matrix<unsigned int> SparseCooccurenceMatrix;
//Update again - this is waay too slow. Need something with O(1) update cost. Going back to heap idea, with external storage.

//Functors for assigning a unique ID to a heap entry
//Required by boost relaxed_heap and mutable_queue for storing non-built-in types
/*
class EdgeWithParams_IDmaker
{
public:
    bool operator() (const EdgeWithParams &e) const
    {
        return //Have to make some ID here. Relaxed heap annoying stores IDs in a flat structure
                //So you have to
                // a) Know the maximum number of things that will be in the heap ahead of time
                // limit the IDs to this range

                //This is unacceptable.
                //We'll have to find some other heap implementation
                //Actually, for the bit where we need a priority heap, 
                //we only ever update the thing we just popped
                //So the STL priority queue should be OK.
    }
};
*/

//Define priorty queue type
//typedef boost::relaxed_heap<NodeWithOccurences, NodeWithOccurences_Comparator_LargestFirst> PriorityHeap;




#endif //ACC_CHOW_LIU_SPARSE_CONTAINERS_H
