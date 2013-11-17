#ifndef ACC_CL_KRUSKAL_H
#define ACC_CL_KRUSKAL_H

#include <vector>

/**
   Represents the union-find structures for Kruskal's algorithm (for determining a Minimum Spanning Tree).
*/
template <typename EdgeType>
class Kruskal
{
 public:
    /**
       Computes a MST of a graph.
       @param numberOfNodes The number of nodes in the graph
    */
    Kruskal(unsigned int nNumOfNodes, unsigned int nNumberOfExtraConnectedComponents = 0)
        : m_nNumberOfNodes(nNumOfNodes), m_nNumEdgesExpected(nNumOfNodes -1 - nNumberOfExtraConnectedComponents)
    {
       // initialize
       initUnionFind();
       MST.reserve(m_nNumberOfNodes-1);
    }
    
    //** Returns true if the set of edges does not yet form a single connected spanning tree. */
    bool notComplete();

    /** Attempts to add an edge. If the edge does not create a cycle, it is added to the MST. */
    void ConsiderEdge(EdgeType edge);

    /** The result - a list of the edges that form the MST. */
    std::vector<EdgeType> MST;

 private:    
    /** Initializes the union/find data structure. */
    void initUnionFind();

    /** Number of nodes in the graph whose MST is being computed */
    unsigned int m_nNumberOfNodes;
    unsigned int m_nNumEdgesExpected; //Maybe be less than m_nNumberOfNodes-1 if the input graph is not fully connected.

    //**************************//
    //STRUCTURES FOR UNION-FIND
    //**************************//

    /**
       Performs a find operation.
       Path compression is applied.
       @param node the identifier of the node whose set should be determined
       @return the identifier of the canonical node which represents the set
               which "node" belongs to
    */
    NodeID find(NodeID node);

    /**
       Performs a union operation.
       @return true iff node1 and node2 have belonged to different sets
    */
    bool unite(NodeID node1, NodeID node2);

    /**
       A vector which contains for each node the identifier of the parent node.
       The canonical node of a set (= the root of a tree) points to itself.
    */
    std::vector<NodeID> _parent;

    /**
       A vector which contains for each node the height of the belonging tree.
       Only the values of canonical nodes are relevant.
    */
    std::vector<char> _height;

};


/**
   Performs a find operation.
   Path compression is applied.
   @param node the identifier of the node whose set should be determined
   @return the identifier of the canonical node which represents the set
           which "node" belongs to
*/
template < typename EdgeType >
inline NodeID Kruskal<EdgeType>::find(NodeID node)
{
    // find the root
    NodeID root = node;
    while (_parent[root] != root) root = _parent[root];

    // apply path compression
    NodeID tmp;
    while (_parent[node] != node) {
    tmp = _parent[node];
    _parent[node] = root;
    node = tmp;
    }
    
    return root;
}

/**
   Performs a union operation.
   @return true iff node1 and node2 have belonged to different sets
*/
template < typename EdgeType >
inline bool Kruskal<EdgeType>::unite(NodeID node1, NodeID node2)
{
    NodeID root1 = find(node1);
    NodeID root2 = find(node2);

    // A cycle would be produced. Therefore the union operation is cancelled.
    if (root1 == root2) return false;

    // Add the smaller tree to the bigger tree
    if (_height[root1] < _height[root2]) {
    _parent[root1] = root2;
    }
    else {
    _parent[root2] = root1;
    
    // Increment the height of the resulting tree if both trees have
    // the same height
    if (_height[root1] == _height[root2]) _height[root1]++;
    }
    return true;
}


/** Initializes the union/find data structure. */
template < typename EdgeType >
void Kruskal<EdgeType>::initUnionFind()
{
    // init parent vector
    _parent.resize(m_nNumberOfNodes);
    for (unsigned int i=0; i<_parent.size(); i++) _parent[i] = i;

    // init height vector
    _height.resize(m_nNumberOfNodes);
}

template < typename EdgeType >
void Kruskal<EdgeType>::ConsiderEdge(EdgeType edge)
{
    // perform union operation according to the current edge
    if (unite( edge.a, edge.b )) 
    {
        // The current edge doesn't produce a cycle.
        // Hence, add it to the resulting MST.
        MST.push_back(edge);
    }
}

template < typename EdgeType >
bool Kruskal<EdgeType>::notComplete()
{
    return MST.size() != m_nNumEdgesExpected;
}

#endif // ACC_CL_KRUSKAL_H
