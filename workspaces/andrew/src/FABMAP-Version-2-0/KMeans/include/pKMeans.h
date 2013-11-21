//***************************************
//    Approximate Out-Of-Core KMeans    *
//            Mark Cummins              *
//     Oxford Mobile Robotics Group     *
//               2008/3/30              *
//***************************************

#ifndef _PKMEANS_H_
#define _PKMEANS_H_

#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <MOOSGenLib/MOOSGenLibGlobalHelper.h>
#include "nn.h" //KD-Tree
#include <boost/random.hpp>
#include <ctime>

using namespace std;


class ClusterCentre
{
public:
    ClusterCentre(){}

    ClusterCentre(unsigned int nDimension)
        : pos(nDimension,0.0f), sum_of_point_positions(nDimension,0.0), num_points(0)
    {}

    ClusterCentre(const vector<float> &point)
        : pos(point), num_points(1)
    {
        unsigned int nDimension = point.size();
        sum_of_point_positions.reserve(nDimension);
        for(unsigned int i=0;i<nDimension;i++)
            sum_of_point_positions.push_back((double)point[i]);
    }

    vector<float> pos;                     // pos = sum_of_point_positions/num_points
    vector<double> sum_of_point_positions; //Sum of the positions of all the points in the cluster
    unsigned int   num_points;             //Number of points in the cluster

    //Sigma_p and n_p allows for efficient updating of the mean of the cluster as points are added or removed.
    //
    //Numerical precision may become an issue here, depending on how many points are in the cluster, 
    //and what range of values the points to be clustered take on. 
    //For SURF descriptors, I think we can probably deal with at least 10^10 points per cluster with acceptable precision.
    //That should still give about 5 digits of precision in each coordinate.
};

//*************************************
//   Define container types
//  Select external/internal memory
//*************************************
#ifdef USE_STXXL_OUT_OF_CORE_CONTAINERS
    //Parameters of external memory containers
    //For a discussion of parameters, see http://algo2.iti.uni-karlsruhe.de/dementiev/files/stxxl_tutorial.pdf
    //Total internal memory consumption is NO_OF_PAGES*PAGE_SIZE*BLOCK_SIZE bytes
    //More internal memory means faster performance
    const unsigned int PEXT_DEFAULT_NO_OF_PAGES = 64;
    const unsigned int AEXT_DEFAULT_NO_OF_PAGES = 32;
    const unsigned int EXT_DEFAULT_PAGE_SIZE = 4;
    const unsigned int EXT_DEFAULT_BLOCK_SIZE = 1*1024*1024;
    #include <stxxl/vector>
    typedef stxxl::VECTOR_GENERATOR<float,EXT_DEFAULT_PAGE_SIZE,PEXT_DEFAULT_NO_OF_PAGES,EXT_DEFAULT_BLOCK_SIZE>::result PointContainer;
    typedef stxxl::VECTOR_GENERATOR<unsigned int,EXT_DEFAULT_PAGE_SIZE,AEXT_DEFAULT_NO_OF_PAGES,EXT_DEFAULT_BLOCK_SIZE>::result AssignmentsConatiner;
#else
    typedef std::vector<float> PointContainer;
    typedef std::vector<unsigned int> AssignmentsConatiner;
#endif
typedef std::vector<ClusterCentre> CentresContainer;

class KMeans
{
public:
    KMeans(const char * i_sMissionFile)
        :m_sMissionFile(i_sMissionFile),
        rng(static_cast<unsigned int>(std::time(0)))       //Initialize random number generator
    {}
    void Run();
private:
    //Basic functions
    void ReadConfigurationParameters();
    void LoadInputData();
    void WriteResults();   
    void RecordConfigParamsToFile(string output_path);
    float CalculateSSE();
    void CallKMeans();
    void RunApproxOutOfCoreKMeans();    //This is our own code. Designed to scale to >10^9 points, >100GB of data. Uses STXXL for external memory containers, and David Lowe's randomized kd-trees for efficient nearest neighbour finding.
    void RunMpiKMeans();                //KMeans using the MPI library from mloss.org. This is exact, an quick, but uses O(N*K) memory. Useful as a reference implementaion on small datasets.

    //File IO
    //void read_data_from_file(std::string &sFile,vector<double> &data,unsigned int &dim);
    //void read_data_from_directory(std::string &sDirectory,std::string &sFileType,vector<double> &data,unsigned int &dim);
    //bool ParseFile(const std::string &sFile,vector<double> &data,unsigned int &nDescriptorDimension);

    //Functions specific to ApproxOutOfCoreKMeans
    void RemovePointFromCluster(unsigned int nClusterID,const vector<float> &Point);
    void AddPointToCluster(unsigned int nClusterID,const vector<float> &Point);
    void UpdateClusterCentroid(unsigned int nClusterID);
    void MergeClusters(unsigned int nClusterID1, unsigned int nClusterID2);
    bool HasTerminated(unsigned int nPointsChanged, unsigned int nIterations);
    bool SSEHasConverged(float fSSE, float fSSELastIteration);

    //Cluster Initialization
    void InitializeClusterCentres(); 
    void RandomInitialization();
    void RadiusBasedInitialization();
    void RadiusBasedInitializationWithKDTree();
    void RandomKSubset(const unsigned int k, const unsigned int n, vector<unsigned int> &result);
    float m_fInitializationRadius;

    //Clustering Parameters
    unsigned int m_nDim;
    unsigned int m_nNumPts;
    unsigned int m_nNumClus;

    //Stats
    float m_fInitialSSE;
    unsigned int m_nNumClustersSuggestedByInitializationMethod;
    unsigned int m_nIterationsCompleted;
    float m_fFinalSSE;

    //Where to read and save the data
    string m_sMissionFile;
    string m_sDataPath;
    string m_sFileType;
    string m_sSubdirName;
    bool m_bReadFromSingleFile;
    bool m_bReadFromDirectoryTree;
    string m_sFile;
    bool m_bFilterInputDataOnBlobResponse;
    double m_dfBlobFilterThreshold;
    string m_sOutputPath;
    bool m_bOXVOutputFormat;
    bool m_bWriteAssignments;
    bool m_bUniqueOutputSubdir;
    bool m_bPrintSSE;
    double m_dfStartTime;

    //Options
    bool m_bRunExactKMeans;
    bool m_bRandomInitialization;
    bool m_bMergeCloseClusters;
    double m_dfClusterMergingThreshold;
    unsigned int m_nNumRestarts;

    //Convergence
    unsigned int m_nMaxIterations;  //0 means iterate until convergence
    double m_dfMaxRunTime;          //Terminate after running for this many seconds.
    float m_fSSEConvergenceThreshold; //Terminate if SSE changes by less than this in some iteration. 
    bool m_bUseSSEbasedConvergenceTest;

    //Refinement stage
    bool m_bDoRefinementStep;   //When convergence is first detected, increase m_nMaxKDTreeNodesToCheck and see if the SSE will decline further.
    unsigned int m_nNodesToCheckMultiplier;
    unsigned int m_nRefinementCycles;

    //kdTree options
    unsigned int m_nNumKDTrees;
    unsigned int m_nMaxKDTreeNodesToCheck;   //This is the key parameter which determines the computation. More gives more accurate nearest neighbours.

    PointContainer Points;
    CentresContainer Centres;
    AssignmentsConatiner Assignment;

    //Random number generator
    //We're using Boost for high quality random numbers
    //Repeat period of 2^32000 instead of 2^15 for standard rand(). It's also fast.
    boost::lagged_fibonacci607 rng;
};

#endif //_PKMEANS_H_
