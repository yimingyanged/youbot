//*******************************************************
//           Accelerated Chow Liu Learning              *
//                   Mark Cummins                       *
//            Oxford Mobile Robotics Group              *
//                    2008/3/12                         *
//*******************************************************

#include "AcceleratedCL.h"
#include "ProgressMeter.h"
#include "GlobalOperators.h"
#include "kruskal.h"
#include <fstream>
#include "IOHelperFunctions.h"


AcceleratedCLCalculator::AcceleratedCLCalculator(string i_path, string i_datasetName, string i_output_path,double i_dfBlobResponseThreshold)
:
path(i_path),datasetName(i_datasetName),output_path(i_output_path),dfBlobResponseThreshold(i_dfBlobResponseThreshold)
{
    //Ask the parser to set num_scenes and vocab_size
    ParseOXS_PeekDimensions(path, datasetName + ".oxs",m_num_scenes, m_vocab_size);
    m_dfNumScenes = (double) m_num_scenes;   //For efficiency we also store a double-valued num_scenes
    m_dfOneOverNumScenes = 1.0/m_dfNumScenes;

    //Initialize the containers
    m_occurences = vector<unsigned int>(m_vocab_size,0);
    m_C = vector<vector<NodeWithMutualInfo> >(m_vocab_size);
    m_V0bar = vector<vector<NodeID> >(m_vocab_size);
    m_Loffset = vector<unsigned int>(m_vocab_size);
    m_NodeIDtoLindex = vector<unsigned int>(m_vocab_size);
    m_parents = vnl_vector<int>(m_vocab_size,-1); 
    
}

void AcceleratedCLCalculator::DoCalc()
{
    //Time the execution
    double dfStartTime = HPMOOSTime(); 

    //Set up utility data structures
    MakeSparseCooccurenceStructures();
    ComputeCLTree();

    //Write out the results
    WriteToFile(m_parents, "ChowTree", datasetName, output_path);
    WriteToFile(m_RelevantConditionals, "RelevantConditionals", datasetName, output_path);
    WriteToFile(m_RelevantNotConditionals, "RelevantNotConditionals", datasetName, output_path);
    WriteToFile(m_Marginals, "Marginals", datasetName, output_path);
    //RecordNumTrainingSamples();

    //Finish
    double timeTaken = HPMOOSTime() - dfStartTime;
    if(timeTaken > 60.0)
    {
        cout << "Finished. Total time " << setprecision(3) << ((timeTaken - fmod(timeTaken,60.0))/60.0) << " minutes " 
            << fmod(timeTaken,60.0) << " seconds." << endl;
    }
    else
    {
        cout << "Finished. Total time " << timeTaken  << " seconds." << endl;
    }

}

void AcceleratedCLCalculator::ComputeCLTree()
{
    //Invariant
    //For each node, the edge with largest mutual information
    //that originates at that node and terminates at a node with lower Nv
    //and that hasn't already been considered by Kruskal's alg
    //is in the heap

    //So, first fill the heap with the largest such edge for each node
    cout <<  endl << "Starting to compute the Chow Liu tree." << endl;

    PriorityHeap edgeheap;
    unsigned int nOrphanNodes = 0;
    for(unsigned int i=0;i<m_vocab_size;++i)
    {
        if(m_occurences[i] != 0)    //Defend against empty clusters. Some "orphan" nodes occur 0 times. Thus they have 0 mutual info with all other nodes. Adding them to the graph means the Kruskal's alg will not terminate until almost every edge in the (complete) graph has been examined. So exclude them!
        {
            if((m_C[i].size() != 0) || (m_V0bar[i].size() < (m_vocab_size - m_Loffset[i])))
            {
                EdgeWithParams e = GetEdgeWithLargestMutualInfo(i);
                edgeheap.push(e);
            }
            else
            {
                // cout << "Skipped node " << i << endl;
            }
        }
        else
        {
            ++nOrphanNodes;
        }
    }

    //Then, process the heap
    //until we get a minimum spanning tree, 
    //or we run out of edges to process (which might happen if the input graph is not connected).
    //Every time we remove an edge from the heap, add 
    //another edge from the appropriate node, if one exists.

    cout << "Running Kruskal's Algorithm..." << endl;

    ProgressMeter progress(m_vocab_size-nOrphanNodes);
    Kruskal<EdgeWithParams> kruskal(m_vocab_size,nOrphanNodes);
    while(kruskal.notComplete() && !edgeheap.empty())
    {
        //Get the largest unconsidered edge
        EdgeWithParams largest_edge = edgeheap.top();
        edgeheap.pop();

        //Process it
        kruskal.ConsiderEdge(largest_edge);
        
        //Now, requeue the next largest edge for the relevant node, if any
        if((m_C[largest_edge.a].size() != 0) || (m_V0bar[largest_edge.a].size() < (m_vocab_size - m_Loffset[largest_edge.a])))
        {
            edgeheap.push(GetEdgeWithLargestMutualInfo(largest_edge.a));
        }
        progress.WriteProgressLowerTriangle(kruskal.MST.size());
    }

    //Done. Output it.
    OutputCLTree_AndCalculateMarginalsAndConditionals(kruskal.MST);
}

void AcceleratedCLCalculator::OutputCLTree_AndCalculateMarginalsAndConditionals(const vector<EdgeWithParams> &MST)
{
    //We want the MST as a vector of the parent of each node
    //This function takes an MST as a list of edges, and fills in the m_parents array
    //It also calculates the relevant marginal and conditional probabilities, 
    //because, due to the data structures, the two calculations are hard to disentangle.
    cout << "\nConverting MST edge list into parent list..." << endl;
 
    //First, initialize the Marginal and conditional arrays
    m_Marginals = GetMarginals();
    m_RelevantConditionals = m_Marginals;   //Conditionals are initialized to marginals. Nodes that aren't the route or orphans will have their values reset later.
    m_RelevantNotConditionals = 1.0 - m_Marginals;

    //First convert the edge list to an adjacency list
    vector<MarkedNode> adjacency_list(m_vocab_size,MarkedNode());
    for (unsigned int i=0; i<MST.size(); i++)
    {
        NodeID source = MST[i].a;
        NodeID target = MST[i].b;
        double dfConditionalProbAB = GetConditionalProb(m_occurences[MST[i].a],m_occurences[MST[i].b],MST[i].nN_ab,false);
        double dfNegativeConditionalProbAB = GetConditionalProb(m_occurences[MST[i].a],m_occurences[MST[i].b],MST[i].nN_ab,true);

        adjacency_list[source].neighbours.push_back(target);
        adjacency_list[source].conditionals.push_back(dfConditionalProbAB);
        adjacency_list[source].negative_conditionals.push_back(dfNegativeConditionalProbAB);
        
        double dfConditionalProbBA = GetConditionalProb(m_occurences[MST[i].b],m_occurences[MST[i].a],MST[i].nN_ab,false);
        double dfNegativeConditionalProbBA = GetConditionalProb(m_occurences[MST[i].b],m_occurences[MST[i].a],MST[i].nN_ab,true);

        adjacency_list[target].neighbours.push_back(source);
        adjacency_list[target].conditionals.push_back(dfConditionalProbBA);
        adjacency_list[target].negative_conditionals.push_back(dfNegativeConditionalProbBA);
    }
    cout << "\nDone making adjacency list..." << endl;

    //Now we do a depth first serch from some arbitrary start node, and set the parent pointers of each node as we visit it.
    NodeID start_node_id = 0;  int node_parent = -1;
    while(adjacency_list[start_node_id].neighbours.size() == 0) {start_node_id++;}    //Needed due to empty clusters again. The root can be any node, except an orphan.
    cout << "(parent tree rooted at node " << start_node_id << ")" << endl;
    DfsVisitOrder(start_node_id,node_parent,m_parents,m_RelevantConditionals,m_RelevantNotConditionals,adjacency_list);

    //Write result to disk
    cout << "Logging mutual information values" << endl;
    std::ofstream outFile((output_path + datasetName + ".mst").c_str());
    outFile << MST;
}

EdgeWithParams AcceleratedCLCalculator::GetEdgeWithLargestMutualInfo(const NodeID &n)
{
    //Returns the edge with largest mutual information
    //that originates at node n and terminates at a node with lower Nv
    //given that there is some valid edge originating at n.
    //Removes the relevant entries from the C or V list.

    EdgeWithParams e;
    e.a = n;

    //Edge we want is either the head of C, or the head of V0
    //Head of V0
    //First node that is a successor of u, and not in VObar
    MutualInfo dfI_V0 = -1.0;
    NodeID nV0_ID;
   
    //Find the first following entry in m_L not in V0bar, and with number of occurences strictly less than n
    for(unsigned int i=m_Loffset[n];i<m_vocab_size;++i)
    {
        //Once we consider this offset, increment it, so we never consider it again.
        m_Loffset[n] += 1;

        //Check if it's in m_V0bar
        //m_V0bar is sorted with the largest entry last.
        if((m_V0bar[n].size() == 0) || (m_L[i].nID != m_V0bar[n][m_V0bar[n].size()-1]))
        {
            //m_L[i] is acceptable. Return it
            dfI_V0 = GetMutualInfo(m_num_scenes,m_occurences[n],m_L[i].nNv,0); //Nuv is 0, becuase nodes listed in V0 do not cooccur with node u.
            nV0_ID = m_L[i].nID;
            break;
        }
        else
        {
            //m_L entry was excluded because it matched the head of m_V0bar
            //We can now remove the head of m_V0bar
            m_V0bar[n].resize(m_V0bar[n].size()-1);
        }    
    }
    
    //Head of C
    NodeWithMutualInfo HeadOfC;
    HeadOfC.I = -1.0;
    if(m_C[n].size()>0)
    {
        //C is sorted with the largest entry last, so the head of C is C[end]
        HeadOfC = m_C[n][m_C[n].size()-1];
    }
    if(HeadOfC.I > dfI_V0) 
    {
        //Edge with head of C
        e.b = HeadOfC.nID;
        e.I = HeadOfC.I;
        e.nN_ab = HeadOfC.nN_uv;

        //Update C list - remove this node
        m_C[n].resize(m_C[n].size()-1);
    }
    else
    {
        //Edge with head of V0
        e.b = nV0_ID;
        e.I = dfI_V0;
        e.nN_ab = 0;
    }

    return e;
}

class TranslateSparseCooccurencesIntoVList : public unary_function<Cooccurence,void>
{
public:
    TranslateSparseCooccurencesIntoVList(vector<vector<NodeWithMutualInfo> > &m_C,vector<vector<NodeID> > &m_V0bar,vector<unsigned int> &m_occurences,unsigned int N,Cooccurence FirstInVector )
        : C(m_C), V0bar(m_V0bar), occurences(m_occurences), nN(N),  LastCooccurence(FirstInVector), nCC(0), nProcessed(0)
    {}
    
    unsigned int operator() (Cooccurence ThisCooccurence)
    {
        ++nProcessed;
        if(nProcessed % 1000000 == 0)
        {
            cout << nProcessed/1000000 << "...";
        }

        if(ThisCooccurence == LastCooccurence)
        {
            ++nCC;
        }
        else
        {
            //Commit to m_C and m_V0bar here
            InsertIntoLists();
            //Reset counter for new Cooccurence
            LastCooccurence = ThisCooccurence;
            nCC = 1;
        }
        return nCC;
    }

    void InsertIntoLists()
    {
        //First, add to V list
        //provided it occurs fewer times than the relevant node. 
        if(occurences[LastCooccurence.nID2] < occurences[LastCooccurence.nID1])
        {
            V0bar[LastCooccurence.nID1].push_back(LastCooccurence.nID2);
        }

        //Now, get mutual info and insert into C list
        NodeWithMutualInfo nm;
        nm.nID = LastCooccurence.nID2;
        nm.I = GetMutualInfo(nN,occurences[LastCooccurence.nID1],occurences[LastCooccurence.nID2],nCC);
        nm.nN_uv = nCC;
        C[LastCooccurence.nID1].push_back(nm);
    }
private:
    //References to m_C and m_V0bar containers
    vector<vector<NodeWithMutualInfo> > &C;
    vector<vector<NodeID> > &V0bar;
    //Variable propreties
    vector<unsigned int> &occurences;
    const unsigned int nN;   //Number of scenes
    //Counter
    Cooccurence LastCooccurence;
    unsigned int nCC;
    unsigned int nProcessed;
};


void AcceleratedCLCalculator::MakeSparseCooccurenceStructures()
{
    CooccurenceContainer cocontainer;
    //Block for memory management
    //After processing the block, all we need to keep in memory is co-container
    {
        //Create the inverted index from the .OXS file
        vector<vector<unsigned int> > WordToScenesIndex = vector<vector<unsigned int> >(m_vocab_size); //An index that stores which scenes a word occured in
        bool goodRead = ParseOXStoInvertedIndex(path + datasetName + ".oxs", WordToScenesIndex,dfBlobResponseThreshold);
        MOOSTrace("Dataset has %d scenes, %d words\n",m_num_scenes,m_vocab_size);

        //Create list L
        //list of words, with how often they occur
        //sorted in order of descending number of occurences
        NodeWithOccurences word;
        for(unsigned int w=0;w<m_vocab_size;++w)
        {
            word.nID = w;
            word.nNv = WordToScenesIndex[w].size();
            m_L.push_back(word);
        }
        sort(m_L.begin(),m_L.end(),NodeWithOccurences_Comparator_LargestFirst());

        //Create NodeID to L index mapping
        for(unsigned int w=0;w<m_vocab_size;++w)
        {
            m_NodeIDtoLindex[m_L[w].nID] = w;
        }

        cout << endl << "Populating the C,L and V lists:" << endl << endl;
        //We also maintain a mapping from NodeID to number of occurences
        //This is just the same as L, but sorted by node ID.
        CalculateOccurencesCache(WordToScenesIndex);

        //Fetch a forward index
        cout << "Creating forward index... ";
        vector<vector<NodeWithOccurences> > ScenesToWordsIndex;
        CreateForwardIndexFromInverted(ScenesToWordsIndex,WordToScenesIndex);
        cout << "done." << endl;

        //Create list V0bar
        //Each entry V0bar(u) stores those words for which N_v < N_u and N_uv > 0.
        //Each entry is sorted in decresing order of N_v
        cout << endl << "Creating V list: " << endl;
        cout << "  making cooccurence container..." << endl << endl;

        //Calculate the (sparse) co-occurence matrix
        //Meila suggests a heap-based idea here
        // -having tried it, it is very wasteful of memory
        // -would be OK with a decent implementaion of Fibonacci heaps
        //  but the one in Boost is broken, and the relaxed_heap structure is inefficient
        //  because of the implementaion of UpdateKey().
        //Update - tried replacing with sparse matrices.
        //This is horribly slow, and liable to run out of memory
        //Really need O(1) add.
        //Update 2 -
        //Switched to Meila's heap idea, implemented as an STXXL vector in external memory.
        //This turns out to be the best thing after all.

        ProgressMeter progress(ScenesToWordsIndex.size());
        cout << "Processing " << m_num_scenes << " scenes...";
        unsigned int nNumComitted = 0;
        for(unsigned int s=0;s<m_num_scenes;s++)
        {
            vector<NodeWithOccurences> scene = ScenesToWordsIndex[s];
            unsigned int nNumWordsInScene = scene.size();
            for(unsigned int w1 =0;w1<nNumWordsInScene;w1++)
            {
                for(unsigned int w2=w1+1;w2<nNumWordsInScene;w2++)
                {
                    //Ensure that we always order the node IDs consistently
                    unsigned int word1 = scene[w1].nID;
                    unsigned int word2 = scene[w2].nID;
                    if((m_occurences[word1] > m_occurences[word2]) || (m_occurences[word1] == m_occurences[word2] && word1 > word2))
                    {
                        Cooccurence c; 
                        c.nID1 = word1;     c.nID2 = word2;
                        cocontainer.push_back(c);
                    }
                    else
                    {
                        Cooccurence c; 
                        c.nID1 = word2;     c.nID2 = word1;
                        cocontainer.push_back(c);
                    }
                    ++nNumComitted;
                }
            }

            //Let the user know about our progress
            if(s%250==0)
            {
                cout << s << "...";
            }
            progress.WriteProgressLinear(s);
        }

        cout << endl << "Sent " << nNumComitted << " cooccurences to file" << endl;
        cout << "STXXL reports " << cocontainer.size() << " entries" << endl;
    }

    //Now, sort the conatiner
    cout << endl << endl << "  sorting..." << endl;
    stxxl::sort(cocontainer.begin(),cocontainer.end(),Cooccurence_Comparator_SmallestFirst(),EMC_INTERNAL_MEMORY_FOR_SORTING);

    //Unroll the co-occurence containaer, making V and C lists
    cout << "  unrolling to V and C lists..." << endl;
    if(cocontainer.size() > 0)
    {
       TranslateSparseCooccurencesIntoVList scan_helper = stxxl::for_each(cocontainer.begin(),cocontainer.end(),TranslateSparseCooccurencesIntoVList(m_C,m_V0bar,m_occurences,m_num_scenes, cocontainer[0]),EMC_DEFAULT_NUM_BLOCKS_FOR_SCAN);
       scan_helper.InsertIntoLists();  //for_each doesn't cause the very last co-occurence to be inserted. Do it here.
    }

    //Sort V and C lists
    cout << "  sorting V and C lists..." << endl;
    for(unsigned int i=0;i<m_vocab_size;++i)
    {
        std::sort(m_C[i].begin(),m_C[i].end(),NodeWithMutualInfo_Comparator_LargestLast());
    }
    for(unsigned int i=0;i<m_vocab_size;++i)
    {
        std::sort(m_V0bar[i].begin(),m_V0bar[i].end(),NodeIDComparator_SortByOccurences_LargestLast(m_occurences));
    }
    
    //Initialize m_Loffset[i] to point to the first element of m_L
    //that occurs strictly fewer times than node i.
    for(unsigned int i=0;i<m_vocab_size;++i)
    {
        unsigned int offset = m_NodeIDtoLindex[i]+1;    //Must be greater than this, becuase m_L is sorted by occurences
        while( offset < m_L.size() && !(m_L[offset].nNv < m_occurences[i]))
        {
            ++offset;
        }
        m_Loffset[i] = offset;
    }


    cout << "  done." << endl;
}

void AcceleratedCLCalculator::CreateForwardIndexFromInverted(vector<vector<NodeWithOccurences> > &ScenesToWordsIndex,const vector<vector<unsigned int> > &WordToScenesIndex) const
{
    //Creating a forward index from the inverted index
    //An index from Scenes to Words
    //given an index from Words to Scenes
    //Assumes that WordToScenesIndex and m_occurences have already been initialized
    
    ScenesToWordsIndex = vector<vector<NodeWithOccurences> >(m_num_scenes);

    NodeWithOccurences node;
    for(unsigned int w=0;w<m_vocab_size;w++)
    {
        node.nID = w;
        node.nNv = m_occurences[w];

        unsigned int nNumScenesContaining = WordToScenesIndex[w].size();
        for(unsigned int s=0;s<nNumScenesContaining;s++)
        {
            ScenesToWordsIndex[WordToScenesIndex[w][s]].push_back(node); //m_WordToScenesIndex[w][s] is the SceneID
        }
    }
    //Sort the words in each scene in order of N_v (number of occurences in the whole dataset)
    for(unsigned int s=0;s<m_num_scenes;s++)
    {
        sort(ScenesToWordsIndex[s].begin(),ScenesToWordsIndex[s].end(),NodeWithOccurences_Comparator_LargestFirst());
    }

}

void AcceleratedCLCalculator::CalculateOccurencesCache(const vector<vector<unsigned int> > &WordToScenesIndex)
{
    //We'll calc a vector such that occurences[w][s]
    //is the number of occurences of word w in state s
    //It's useful to cache this as we'll need it many times

    for(unsigned int w=0;w<m_vocab_size;w++)
    {
        m_occurences[w] = WordToScenesIndex[w].size();
    }
}





//FIX ME. This function not correct. Nsamples for each conditional is different.
void AcceleratedCLCalculator::RecordNumTrainingSamples() const
{
    //Record how many samples were used to train the model.
    //Necessary to know for updating the model later.
    string sOutPath = EnsurePathHasTrailingSlash(output_path);
    ofstream fFile((sOutPath + "NTrainingSamples.txt").c_str(),ios::out);
    if(!fFile.is_open())
    {
        cerr << "Failed to create NTrainingSamples output file" << endl;
    }
    else
    {
        fFile << "Num_Training_Samples_For_Marginals = " << m_num_scenes << endl;
        double N = (double)m_num_scenes;
        fFile << "Num_Training_Samples_For_RelevantConditionals = " << (N + sqrt(N)) << endl;
        fFile << "Num_Training_Samples_For_RelevantNotConditionals  = " << (N + sqrt(N)) << endl;
    }
    fFile.close();
}

/*
//This is the MLE estimate for Marginals.
vnl_vector<double> AcceleratedCLCalculator::GetMarginals() const
{
    double N = (double) m_num_scenes;
    vnl_vector<double> marginals(m_vocab_size);
    for(unsigned int i=0;i<m_vocab_size;++i)
    {
        marginals[i] = m_occurences[i]/N; //estimate p(a) as (#observations of a)/(total #observations)
    }
    return marginals;
}
*/


//This is the PseudoBayes smoothed estimate for Marginals.
vnl_vector<double> AcceleratedCLCalculator::GetMarginals() const
{
    double N = (double) m_num_scenes;
    double K = sqrt(N);     //K for PseudoBayes Minimax Conditionals

    //Weighting between MLE and prior
    double p_weight = K/(N+K);
    double mle_weight = N/(N+K);

    //Our prior for the marginals. Just a rough guess.
    double prior = GetMeanMarginal();

    vnl_vector<double> marginals(m_vocab_size);
    for(unsigned int i=0;i<m_vocab_size;++i)
    {
        marginals[i] = mle_weight*(m_occurences[i]/N) + p_weight*prior;
    }
    return marginals;
}

double AcceleratedCLCalculator::GetMeanMarginal() const
{
    //Returns the mean marginal, across all words.
    //This is used as the pseudo-Bayesian prior for smoothing the marginals.
    unsigned int nTotalOccurences = 0;
    for(unsigned int i=0;i<m_vocab_size;++i)
        nTotalOccurences += m_occurences[i]; 

    return ((double)nTotalOccurences)/((double)(m_vocab_size*m_num_scenes));    //estimate mean p(a) as (#observations of any word)/(total #observations)
}

/*
=================================================
 Estimating Marginals and Conditionals from Data
=================================================
If data is sparse, the MLE probability values can be 0 or 1. 
This is not good.
We're going to use a pseudo-Bayes estimator, which has the form:

 p_pb = (N/(N+ K))*p_mle + (K/(N+K))*lambda

Where: 
       p_mle is the maximum likelhood estimate from the data
       N is the number of observations
       lambda is a prior for the value of the probability - generally set
       to something null, like the centre of the probability simplex.
       K is the number of "prior samples" 
          - since in reality we have no prior samples, this is some
          artificial confidence in our prior. The trick of pseudoBayes is
          to estimate K from the data so as to minimize the expected risk
          of our estimate.

Bishop,Fienberg and Holland discusses various formulae for K in great detail.

The minimax solution is to choose K = sqrt(N)
which minimizes the maximum risk over all possible values of p (the true probability)

A possibly better solution they call the p* estimator.
The p* estimate has the property that the ratio between
p_mle and p* is bounded, whereas this is not the case for
the minimax estimator. In practice, this means that the
minimax does poorly when some features have extreme
probability values, whereas the p_pb does not.
K* is given by

K* = (p_mle*(1-p_mle)) / (lambda - p_mle)^2

See "Discrete Multivariate Analysis: Theory and Practice"
Yvonne Bishop, Stephen Fienberg, Paul Holland
*/

double AcceleratedCLCalculator::GetConditionalProb(const unsigned int &nNa, const unsigned int &nNb, const unsigned int &nNab, bool bNegate) const
{
    //Calculate p(a=1 | b =1)
    //or, if bNegate == true, calculae p(a=0 | b=0)
    double dfN = (double) m_num_scenes;
    double dfNa = (double) nNa;
    double dfNb = (double) nNb;
    double dfNab = (double) nNab;

    //After some more testing in Matlab, looks like Minimax may not be such a great choice.
    //Could try some other options here.
    double NObs = bNegate ? (dfN-dfNb) : dfNb;  //How many samples do we have for calculating the probability of interest?
    double K = sqrt(NObs);                      //K for PseudoBayes Minimax Conditionals
    if(K < 1)
        K = 1;  //Defend against case where Nobs is 0.

    //Our prior is independence - p( a | b ) = p(a)
    //So, we need to calculate p(a) values
    double marginal = dfNa/dfN;

    //Weighting between MLE and prior
    double p_weight = K/(NObs+K);
    double mle_weight = NObs/(NObs+K);

    //Now get pseudoBayes Conditionals
    if(bNegate)
    {
        return mle_weight*((dfN-dfNa-dfNb+dfNab)/(dfN-dfNb)) + p_weight*(1.0 - marginal);
    }
    else
    {
        return  mle_weight*(dfNab/dfNb) + p_weight*(marginal);  
    }
}

void DfsVisitOrder(NodeID node,int node_parent,vnl_vector<int> &parents,vnl_vector<double> &RelevantConditionals,vnl_vector<double> &RelevantNotConditionals,vector<MarkedNode> &adjacency_list)
{
    if(!adjacency_list[node].marked)
    {
        //Mark the node as visited
        adjacency_list[node].marked = true;

        //Record the parent
        parents[node] = node_parent;

        //Record the relevant conditional probability
        if(node_parent != -1)
        {
            unsigned int parent_index=0;
            for(;parent_index<adjacency_list[node].neighbours.size();++parent_index)
            {
                if(adjacency_list[node].neighbours[parent_index] == node_parent)
                    break;
            }
            RelevantConditionals[node] = adjacency_list[node].conditionals[parent_index];
            RelevantNotConditionals[node] = adjacency_list[node].negative_conditionals[parent_index];
        }

        //Continue DFS
        for(unsigned int i=0; i<adjacency_list[node].neighbours.size(); ++i)
        {
            DfsVisitOrder(adjacency_list[node].neighbours[i],(int) node,parents,RelevantConditionals,RelevantNotConditionals,adjacency_list);
        }
    }    
}
