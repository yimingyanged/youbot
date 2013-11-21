#ifndef ACC_CHOW_LIU_H
#define ACC_CHOW_LIU_H 1

#include <string>
#include <algorithm>
#include <vector>
#include <iomanip>
#include <cmath>
#include "MOOSGenLib/MOOSGenLibGlobalHelper.h"
using namespace std;

#include "OXS_parser.h"
#include "MAT_parser.h"
#include <vnl/vnl_vector.h> //Only used for output to Matlab format

#include "DataTypes.h"
#include "SparseContainers.h"

//Helper function for output
void DfsVisitOrder(NodeID node,int node_parent,vnl_vector<int> &parents,vnl_vector<double> &RelevantConditionals,vnl_vector<double> &RelevantNotConditionals,vector<MarkedNode> &adjacency_list);

class AcceleratedCLCalculator
{
    public:
        AcceleratedCLCalculator(string i_path, string i_datasetName, string i_output_path, double i_dfBlobResponseThreshold);
        void DoCalc();

    private:
        //Functions
        void MakeSparseCooccurenceStructures();
        void ComputeCLTree();
        void OutputCLTree_AndCalculateMarginalsAndConditionals(const vector<EdgeWithParams> &MST);
        void RecordNumTrainingSamples() const;

        //Helper functions
        void CalculateOccurencesCache(const vector<vector<unsigned int> > &WordToScenesIndex);
        void CreateForwardIndexFromInverted(vector<vector<NodeWithOccurences> > &ScenesToWordsIndex,const vector<vector<unsigned int> > &WordToScenesIndex) const;
        EdgeWithParams GetEdgeWithLargestMutualInfo(const NodeID &n);

        double GetConditionalProb(const unsigned int &nNa, const unsigned int &nNb, const unsigned int &nNab, bool bNegate) const;
        vnl_vector<double> GetMarginals() const;
        double GetMeanMarginal() const;
        
        //Constants
        const string path,datasetName,output_path;
        const double dfBlobResponseThreshold;       //Optionally only load features from file above a certain blob response threshold. For easy testing of the effect of varying the number of interest points per image.
        unsigned int m_num_scenes, m_vocab_size;
        double m_dfNumScenes; 
        double m_dfOneOverNumScenes; //So we can multiply instead of divide.
    
        //Containers 
        vector<NodeWithOccurences> m_L;             //The set of words, 
                                                    //sorted in deceasing order of N_u (their number of occurences).

        vector<vector<NodeWithMutualInfo> > m_C;    //Each entry C(u) stores those words for which N_v < N_u and N_uv > 0. 
                                                    //Each entry is sorted in decresing order of I_uv

        //vector<vector<NodeID> > m_V0bar;          //V0bar(u) is now represented implicitly by the entries of m_C

        vector<unsigned int>  m_Loffset;            //Vlist contains all those entries of m_L after M_Loffset, and not included in V0bar

        vector<int>  m_HeadOfCoffset;      //To support memory saving implementation, we'll let the Clist double as the V0bar list. So, instead of deleting elements from the Clist, we'll just leave them there (to represent V0bar) and move this head index to keep track of where the "head of C list" is.

        vector<unsigned int>  m_RemainingV0barEntires;      //How many entries are there left in the (implicit) V0bar.

        vector<unsigned int> m_NodeIDtoLindex;      //Mapping from NodeID to position in m_L vector.
              
        vector<unsigned int>  m_occurences; // occurences[w] holds the number of occurences of word w in state 1. It's useful to cache this, as we need it many times 

        //The results
        vnl_vector<int> m_parents;
        vnl_vector<double> m_Marginals;
        vnl_vector<double> m_RelevantConditionals;
        vnl_vector<double> m_RelevantNotConditionals;
};

inline MutualInfo GetMutualInfo(const unsigned int &nN, const unsigned int &nNu, const unsigned int &nNv, const unsigned int &nNuv)
{
    //Returns mutual info between variables u and v
    //nN is the total number of observations.
    //nNu is the number of occurences of variable u in state 1
    //nNv is the number of occurences of variable v in state 1
    //nNuv is the number of co-occurences of the two variables

    //Speed considerations
    //unsigned int -> float conversion, as much as 20 clock cycles
    //L1 cache fetch, ~4 clock cycles
    //L2 cache fetch, ~30 clock cycles
    //RAM fetch, hundreds of clock cycles.
    //So probably faster to recast than to store and fetch
    
    double dfN =  (double) nN;
    //Each of these quantities is N times the respective probability
    double p_uv_11 = (double) nNuv;
    double p_uv_10 = (double) nNu - nNuv;
    double p_uv_01 = (double) nNv - nNuv;
    double p_uv_00 = (double) nN - nNu - nNv + nNuv;

    double p_u_1 = (double) nNu;
    double p_u_0 = (double) nN - nNu;
    double p_v_1 = (double) nNv;
    double p_v_0 = (double) nN - nNv;

    double base_conversion = 1.0/log10(2.0);
    double dfLogN = log10(dfN)*base_conversion;

    double mutualInfo = 0;

    //From continuity, we define 0log0 = 0
    if(p_uv_11 != 0)
        mutualInfo += p_uv_11*(dfLogN + base_conversion*log10(p_uv_11/(p_u_1*p_v_1) ));
    if(p_uv_10 != 0)
        mutualInfo += p_uv_10*(dfLogN + base_conversion*log10(p_uv_10/(p_u_1*p_v_0)));
    if(p_uv_01 != 0)
        mutualInfo +=p_uv_01*(dfLogN + base_conversion*log10(p_uv_01/(p_u_0*p_v_1)));
    if(p_uv_00 != 0)
        mutualInfo +=p_uv_00*(dfLogN + base_conversion*log10(p_uv_00/(p_u_0*p_v_0)));
    //Finally, divide by common factor
    mutualInfo /= dfN;

    return (MutualInfo) mutualInfo;
}

#endif //ACC_CHOW_LIU_H
