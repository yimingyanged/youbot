#ifndef ACC_CHOW_LIU_DATATYPES_H
#define ACC_CHOW_LIU_DATATYPES_H 1

typedef unsigned int NodeID;
typedef float MutualInfo;  //Memory is tight. Use float. All we want is an ordering anyway. Percise value is not important.

struct NodeWithOccurences
{
    NodeID nID;
    unsigned int nNv;        //In how many scenes does this word occur?
};

struct NodeWithMutualInfo
{
    NodeID nID;         //This class is only used in the list C
    MutualInfo I;       //So, for a NodeWithMutualInfo stored in C(u)
    unsigned int nN_uv; //the MutualInfo value is implicitly the info
                        //between nID and u.
};

struct EdgeWithParams
{
    NodeID a;
    NodeID b;
    unsigned int nN_ab;     //In how many scenes do a and b co-occur?
    MutualInfo  I;          //Mutual info between a and b
};

//Functors for defining sorting criteria
class NodeIDComparator_SortByOccurences_LargestLast
{
public:
    NodeIDComparator_SortByOccurences_LargestLast(const vector<unsigned int> &m_occurences)
        : occurences(m_occurences)
    {}

    bool operator() (const NodeID &n1, const NodeID &n2) const
    {
        return (occurences[n1] < occurences[n2]) || (occurences[n1] == occurences[n2] && n1 < n2);  //The second clause here is a tie-breaker, which ensures that we get consistent sorted order in different containers for nodes that have the same number of occurences.
    }

private:
    const vector<unsigned int> &occurences;
};


class NodeWithOccurences_Comparator_LargestFirst
{
public:
    bool operator() (const NodeWithOccurences &n1, const NodeWithOccurences &n2) const
    {
        return (n1.nNv > n2.nNv)  || (n1.nNv == n2.nNv && n1.nID > n2.nID);  ;
    }
};

class NodeWithMutualInfo_Comparator_LargestLast
{
public:
    bool operator() (const NodeWithMutualInfo &n1, const NodeWithMutualInfo &n2) const
    {
        return n1.I < n2.I;
    }
};


class EdgeWithParams_Comparator_LargestFirst
{
public:
    bool operator() (const EdgeWithParams &e1, const EdgeWithParams &e2) const
    {
        return e1.I < e2.I;
    }
};


class MarkedNode        
{
//A little helper class to massage the MST output into a convenient format
public:
    MarkedNode()
    {
        marked = false;
    }
    bool marked;
    vector<NodeID> neighbours;
    vector<double> conditionals;          //conditionals[i] is p(this_node = 1,neighbours[i] = 1)
    vector<double> negative_conditionals; //negative_conditionals[i] is p(this_node = 0,neighbours[i] = 0)
};

#endif //ACC_CHOW_LIU_DATATYPES_H
