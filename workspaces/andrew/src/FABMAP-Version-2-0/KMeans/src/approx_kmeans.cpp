//***************************************
//    Approximate Out-Of-Core KMeans    *
//            Mark Cummins              *
//     Oxford Mobile Robotics Group     *
//               2008/3/30              *
//***************************************

#include "pKMeans.h"
#include <limits>
using namespace std;

//Need to
//1 - Initialize cluster centres
// Then iterate:
//2 - For each point, find which cluster centre is closest
//3 - Move the clusters centres to the centroid of the points they own

void KMeans::RunApproxOutOfCoreKMeans()
{
    //Assume that the following have been initiaized:
    //  -ClusterCentres
    //  Assignments (set to a value out of range, so that they will be detected changed on first iteration)

     //************************************
    //        Setup datastructures        
    //************************************
    
    //LNN expects as input float **
    //We don't store our cluster centres like this
    //So need to construct the appropriate structure

    //Nothing in the code must change the length of CentresContainer
    //or the length of any of its elements
    //so as not to invalidate these pointers.

    vector<float*> PointersToCentres;
    for(unsigned int i=0;i<m_nNumClus;i++)
    {
        PointersToCentres.push_back(&(Centres[i].pos[0]));
    }

    //Get an RNG that will return integers randomly distributed over the range of point IDs.
    boost::variate_generator<boost::lagged_fibonacci607&, boost::uniform_int<> > get_random_point_ID(rng,boost::uniform_int<>(0,m_nNumPts-1));

    //Const reference to the points container. When using STXXL, this ensures that buffers are not marked as dirty after a read, so they don't need to be written back to disk.
    const PointContainer& CPoints = Points; 

    bool bDone = false;
    bool bNoMoreMerging = false;
    unsigned int nPointsChanged;
    unsigned int nClustersChanged;
    unsigned int nIterations = 0;
    unsigned int nRefinementCyclesCompleted = 0;
    float fSSELastIteration = 0.0f;

    while(!bDone)
    {
        double dfIterStart = HPMOOSTime();
        nPointsChanged = 0;
        nClustersChanged = 0;
        vector<bool> ClusterWasChanged(m_nNumClus+1,false);

        //************************************
        //  Build kd-tree of cluster centres  
        //************************************
        Index kdtree = BuildIndexFloat(&(PointersToCentres[0]),(int) m_nNumClus,(int) m_nDim,m_nNumKDTrees);
        double dfTreeEnd = HPMOOSTime();
        cout << "Tree build takes " << dfTreeEnd - dfIterStart << endl;

        //*******************************************************
        //              INNER LOOP
        // For each point, find it's nearest cluster centre
        // If this has changed since the last iteration
        //     1) Update affiliation
        //     2) Alter cluster mean
        //*******************************************************
        int nNearestClusterID;
        vector<float> CurrentPoint(m_nDim);
        for(unsigned int pointID=0, vectorOffset=0; pointID<m_nNumPts; ++pointID, vectorOffset+=m_nDim)
        {
            //Construct the query
            //We can't just pass &(CPoints[vectorOffset]) to the kd tree directly, becuase I think pointer-based access is not safe with STXXL. Data may be on disk. Need to access via the [] operator. 
            for(unsigned int i=0,j=vectorOffset;i<m_nDim;i++,j++)
            {
                CurrentPoint[i] = CPoints[j];
            }
            //Query the kd-tree.
            FindNeighborsFloat(&(nNearestClusterID),
                                1,                   //How many near neighbours to find - We just want one
                                &(CurrentPoint[0]),
                                kdtree,
                                m_nMaxKDTreeNodesToCheck);

            //If it changed cluster, update affiliation and cluster means
            if(nNearestClusterID != Assignment[pointID])
            {
                //Update cluster centres
                RemovePointFromCluster(Assignment[pointID],CurrentPoint);
                AddPointToCluster(nNearestClusterID,CurrentPoint);
                //Mark clusters as changed
                ClusterWasChanged[nNearestClusterID] = true;
                ClusterWasChanged[Assignment[pointID]] = true;
                //Change point affiliation
                Assignment[pointID] = nNearestClusterID;
                nPointsChanged++;
            }
        }

        //Record debug info.
        if(nIterations==0)
        {
            //Record SSE. This is only for debugging purposes. Adds a little to computation time, but not excessively.
            cout << "...calculating initial SSE...";
            m_fInitialSSE = CalculateSSE();
            fSSELastIteration = m_fInitialSSE;
            if(m_bPrintSSE)
               cout << endl << "Initial SSE: " << m_fInitialSSE << endl;
        }

        //*******************************************************
        //          Tidy Up
        // Update cluster centres
        // Deal with empty clusters
        // Free kdTree
        // Print diagnostics
        //*******************************************************

        //Update cluster centre positions
        //If a cluster has become empty, re-initialize it to a random datapoint.
        for(unsigned int i=0;i<m_nNumClus;i++)
        {
            if(Centres[i].num_points !=0)
            {
                if(ClusterWasChanged[i])
                {
                    //Update its centroid
                    UpdateClusterCentroid(i);
                    nClustersChanged++;
                }
            }
            else
            {
                //Reassign empty cluster to a random datapoint
                cout << "Reinitialized an empty cluster." << endl;

                unsigned int nRandomPointID = get_random_point_ID();
                unsigned int vectorOffset = m_nDim*nRandomPointID;

                for(unsigned int j=0,k=vectorOffset; j<m_nDim; j++,k++)
                    CurrentPoint[j] = CPoints[k];

                //Remove our random point from its existing cluster.
                RemovePointFromCluster(Assignment[nRandomPointID],CurrentPoint);
                UpdateClusterCentroid(Assignment[nRandomPointID]);

                //Put the random point in the empty cluster
                AddPointToCluster(i,CurrentPoint);
                Assignment[nRandomPointID] = i;
                UpdateClusterCentroid(i);

                nClustersChanged++;
            }

        }

        //Free kd-tree
        FreeIndex(kdtree);


        //Cluster merging
        if(m_bMergeCloseClusters && !bNoMoreMerging)
        {
            //Check if any two cluster centres are closer than the merging threshold
            //If so, merge them, and initialize a new cluster at random
            //This mitigates over-segmentation of dense regions.

            //Rebuild a kd-tree over the new centre locations
            Index centres_kd_tree = BuildIndexFloat(&(PointersToCentres[0]),(int) m_nNumClus,(int) m_nDim,m_nNumKDTrees);

            vector<bool> ClusterNotAlreadyMerged(m_nNumClus,true);

            //For each cluster, find the nearest other cluster
            vector<int> TwoNearestClusterIDs(2,0);
            for(unsigned int nClusterID = 0; nClusterID<m_nNumClus; nClusterID++)
            {
                if(ClusterNotAlreadyMerged[nClusterID])
                {
                    FindNeighborsFloat(&(TwoNearestClusterIDs[0]),
                                        2,                              //How many near neighbours to find - We want two here, because the nearest neighbour for the queary point will be the cluster itself.
                                        &(Centres[nClusterID].pos[0]),
                                        centres_kd_tree,
                                        5*m_nMaxKDTreeNodesToCheck);    //5x - we want results here to be accurate
                    
                    nNearestClusterID = (TwoNearestClusterIDs[0] == nClusterID) ? TwoNearestClusterIDs[1] : TwoNearestClusterIDs[0];       //Nearest point to the query may be cluster itself. If so, take the second nearest point.

                    //If clusters are closer than the threshold, merge them
                    if(ClusterNotAlreadyMerged[nNearestClusterID] && FloatDistSquared(&(Centres[nClusterID].pos[0]),&(Centres[nNearestClusterID].pos[0]),m_nDim) < m_dfClusterMergingThreshold)
                    {
                        MergeClusters(nClusterID,nNearestClusterID);
                        ClusterNotAlreadyMerged[nNearestClusterID] = false; //Prevent any future merges with cluster 2, becuase its position just got randomized. 
                    }
                }
            }
        }

        //Statistics
        nIterations++;
        cout << "Iteration " << nIterations << ". Changed " << nPointsChanged << " points from " << nClustersChanged << " clusters.";
        float fSSE = 0.0f; 
        if(m_bPrintSSE)     //m_bPrintSSE is always true if we are using SSE based convergence test
        {
            fSSE = CalculateSSE();
            cout << " SSE: " << fSSE << endl;
        }
        else
            cout << endl;

        double dfIterationTime = HPMOOSTime() - dfIterStart;
        cout << "Iteration takes " << dfIterationTime << endl;
       
        bool bSSEHasConverged = SSEHasConverged(fSSE,fSSELastIteration);
        
        //Optional refinement step
        if(bSSEHasConverged && m_bDoRefinementStep && nRefinementCyclesCompleted < m_nRefinementCycles)
        {
            //Increase kd-tree search accuracy
            nRefinementCyclesCompleted++;
            m_nMaxKDTreeNodesToCheck *= m_nNodesToCheckMultiplier;
            cout << "Upping KD-tree search accuracy. KDTreeNodesToCheck now at " << m_nMaxKDTreeNodesToCheck  << endl;
            bSSEHasConverged = false;
        }

        fSSELastIteration = fSSE;
        bDone = HasTerminated(nPointsChanged,nIterations) || bSSEHasConverged;

        //Record how many iterations we did before finishing.
        if(bDone)
            m_nIterationsCompleted = nIterations;

        //Disable cluster merging when we are close to the end of the run
        //This ensures we don't terminate with any mixed clusters due to a recent merge/split
        unsigned int nNumMergeFreeFinalIterations = 5;
        if(bSSEHasConverged || nIterations ==  m_nMaxIterations-nNumMergeFreeFinalIterations || (HPMOOSTime() - m_dfStartTime) > (m_dfMaxRunTime-nNumMergeFreeFinalIterations*dfIterationTime))
            bNoMoreMerging = true;

    }
}

bool KMeans::SSEHasConverged(float fSSE, float fSSELastIteration)
{
    bool bSSEConditionMet = false; 
    if(m_bUseSSEbasedConvergenceTest)
    {
        bSSEConditionMet = ((fSSELastIteration-fSSE)/fSSELastIteration)<m_fSSEConvergenceThreshold;
    }

    if(bSSEConditionMet)
        cout << "Converged..." << endl << "Old SSE: " << fSSELastIteration << endl << "New SSE: " << fSSE << endl << "Ratio: " << ((fSSELastIteration-fSSE)/fSSELastIteration) << endl;

    return bSSEConditionMet;
}

bool KMeans::HasTerminated(unsigned int nPointsChanged, unsigned int nIterations)
{
    bool bConvergence = nPointsChanged == 0;
    bool bIterationsCondition = nIterations == m_nMaxIterations;
    bool bTimeCondition = (HPMOOSTime() - m_dfStartTime) > m_dfMaxRunTime;

    if(bConvergence)
        cout << endl << "Converged. Terminating." << endl;

    if(bIterationsCondition)
        cout << endl << "Reached max number of iterations. Terminating." << endl;

    if(bTimeCondition)
        cout << endl << "Reached max running time. Terminating." << endl;

    return bConvergence || bIterationsCondition || bTimeCondition;
}

void KMeans::RemovePointFromCluster(unsigned int nClusterID,const vector<float> &Point)
{
    if(nClusterID < m_nNumClus) //This test is to handle the first iteration where a point's previous assignment is set out of range.
    {
        Centres[nClusterID].num_points--;

        for(unsigned int i=0; i<m_nDim;i++)
        {
            Centres[nClusterID].sum_of_point_positions[i] -= Point[i];
        }
    }
}

void KMeans::AddPointToCluster(unsigned int nClusterID,const vector<float> &Point)
{
    Centres[nClusterID].num_points++;

    for(unsigned int i=0; i<m_nDim;i++)
    {
        Centres[nClusterID].sum_of_point_positions[i] += Point[i];
    }
}

void KMeans::UpdateClusterCentroid(unsigned int nClusterID)
{
    //Update cluster centre positions
    //Since we did most of this work incrementally, all that needs doing is to set 
    // pos = sum_of_point_positions/num_points
    
    double dfNumPtsInCluster = (double) Centres[nClusterID].num_points;
    for(unsigned int j=0;j<m_nDim;j++)
    {
        Centres[nClusterID].pos[j] = (float)(Centres[nClusterID].sum_of_point_positions[j]/dfNumPtsInCluster);
    }
}

void KMeans::MergeClusters(unsigned int nClusterID1, unsigned int nClusterID2)
{
    cout << "   DOING A MERGE between cluster " << nClusterID1 << " and cluster " << nClusterID2 << endl;
    //Assign to cluster 1 all the points of cluster 2
    Centres[nClusterID1].num_points += Centres[nClusterID2].num_points;
    for(unsigned int i=0;i<m_nDim;i++)
        Centres[nClusterID1].sum_of_point_positions[i] += Centres[nClusterID2].sum_of_point_positions[i];
    UpdateClusterCentroid(nClusterID1);

    //Find all points assigned to cluster 2, and assign them to cluster 1 instead
    //This is not efficient becuase we're not maintaining a mapping from a cluster to the points it contains. Should fix this.
    for(unsigned int i=0;i<m_nNumPts;i++)
    {
        if(Assignment[i] == nClusterID2)
        {
            Assignment[i] = nClusterID1;
        }
    }

    
    //Now reinitialize cluster 2
    //First, zero it
    Centres[nClusterID2].num_points = 0;
    Centres[nClusterID2].sum_of_point_positions = vector<double>(m_nDim,0.0);
    Centres[nClusterID2].pos = vector<float>(m_nDim,0.0f);


    //Reassign empty cluster to a random datapoint
    boost::variate_generator<boost::lagged_fibonacci607&, boost::uniform_int<> > get_random_point_ID(rng,boost::uniform_int<>(0,m_nNumPts-1));
    const PointContainer& CPoints = Points; 
    vector<float> CurrentPoint(m_nDim);

    unsigned int nRandomPointID = get_random_point_ID();
    unsigned int vectorOffset = m_nDim*nRandomPointID;

    for(unsigned int j=0,k=vectorOffset; j<m_nDim; j++,k++)
        CurrentPoint[j] = CPoints[k];

    //Remove our random point from its existing cluster.
    RemovePointFromCluster(Assignment[nRandomPointID],CurrentPoint);
    UpdateClusterCentroid(Assignment[nRandomPointID]);

    //Put the random point in the empty cluster
    AddPointToCluster(nClusterID2,CurrentPoint);
    Assignment[nRandomPointID] = nClusterID2;
    UpdateClusterCentroid(nClusterID2);


    
/*
    //An alternative way to re-initialize the empty cluster would be to split an exisiting cluster
    //Rather than selecting a point at random as above
    //Selecting a random datapoint is biased toward over-sgementing dense regions, which is exactly what the merging process is trying to eliminate
    //whereas choosing a cluster at random and splitting it into two should be less biased.

    //Hmmm- It turns out that BuildIndexFloat takes FOR EVER to run if we apply this cluster splitting procedure.
    //I'm not sure why - perhaps it's because the resulting cluster centres are very very close.
    //There is also the danger with this heuristic that we will repeatedly split and then re-merge clusters
    //For the moment, have gone back to picking a random data point to initialize the new cluster
    //That seems to work better.

    boost::variate_generator<boost::lagged_fibonacci607&, boost::uniform_int<> >  get_random_cluster_ID(rng,boost::uniform_int<>(0,m_nNumClus-1));     //An RNG adaptor that will return integers randomly distributed over the range of cluster IDs.
    unsigned int nClusterIDtoSplit = get_random_cluster_ID();   //Maybe could choose highest variance cluster here instead.
    while(nClusterIDtoSplit == nClusterID1 || nClusterIDtoSplit == nClusterID2)
        nClusterIDtoSplit = get_random_cluster_ID();

    cout << "    Reinitializing cluster " << nClusterID2 << " by splitting cluster " << nClusterIDtoSplit << endl;

    //Find all the points in the cluster
    vector<unsigned int> PointsInTheCluster;
    
    for(unsigned int i=0;i<m_nNumPts;++i)
    {
        if(Assignment[i] == nClusterIDtoSplit)
            PointsInTheCluster.push_back(i);
    }

    //Select half of the points at random.
    vector<unsigned int> Random_50percent_subset;
    RandomKSubset(PointsInTheCluster.size()/2,PointsInTheCluster.size(),Random_50percent_subset);

    vector<unsigned int> Point_IDs_to_assign_to_new_cluster;
    for(unsigned int i=0;i<Random_50percent_subset.size();i++)
    {
        Point_IDs_to_assign_to_new_cluster.push_back(PointsInTheCluster[Random_50percent_subset[i]]);
    }

    //Assign these points to a new cluster
    vector<float> CurrentPoint(m_nDim);
    const PointContainer& CPoints = Points; 
    for(unsigned int i=0;i<Point_IDs_to_assign_to_new_cluster.size();i++)
    {
        unsigned int vectorOffset = m_nDim*Point_IDs_to_assign_to_new_cluster[i];
        for(unsigned int j=0,k=vectorOffset; j<m_nDim; j++,k++)
            CurrentPoint[j] = CPoints[k];
        
        RemovePointFromCluster(nClusterIDtoSplit,CurrentPoint);
        AddPointToCluster(nClusterID2,CurrentPoint);
        Assignment[Point_IDs_to_assign_to_new_cluster[i]] = nClusterID2;
    }
    UpdateClusterCentroid(nClusterIDtoSplit);
    UpdateClusterCentroid(nClusterID2);
    */
}
