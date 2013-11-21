#ifndef ACC_CHOW_LIU_GLOBAL_OPERATORS_H
#define ACC_CHOW_LIU_GLOBAL_OPERATORS_H 1
#include "DataTypes.h"
#include "SparseContainers.h"

bool operator==(const Cooccurence &c1,const Cooccurence &c2)
{
    return (c1.nID1 == c2.nID1) && (c1.nID2 == c2.nID2);
}

bool operator==(const NodeWithOccurences &n1,const NodeWithOccurences &n2)
{
    return (n1.nID == n2.nID) && (n1.nNv == n2.nNv);
}

ostream& operator<<(ostream& os,const Cooccurence &c)
{
    return os << "(" << c.nID1 << "," << c.nID2 << ")";
}

ostream& operator<<(ostream& os,const NodeWithOccurences &n)
{
    return os << "(" << n.nID << "," << n.nNv << ")";
}

ostream& operator<<(ostream& os,const NodeWithMutualInfo &n)
{
//    return os << "(" << n.nID << "," << n.I << ")";
    return os << "(" << n.nID << "," << n.nN_uv << ")";
}

ostream& operator<<(ostream& os,const EdgeWithParams &e)
{
    return os << "(" << e.a << "," << e.b << " : " << e.I << ")";
}

//Ostream operator for weighted graphs
ostream& operator<<( std::ostream& os, const vector<EdgeWithParams> &graph) 
{
    os << "Number of nodes: " << graph.size() << endl;

    //Get the total weight of the MST
    MutualInfo dfWeight = 0.0;
    for (unsigned int i=0; i<graph.size(); i++)
        dfWeight += graph[i].I;
    os << "Total weight: " << dfWeight << endl;
    
    for (unsigned int i=0; i<graph.size(); i++)
        os << graph[i].a << " " << graph[i].b << " " << graph[i].I  << endl;
    return os;
}

#endif //ACC_CHOW_LIU_GLOBAL_OPERATORS_H
