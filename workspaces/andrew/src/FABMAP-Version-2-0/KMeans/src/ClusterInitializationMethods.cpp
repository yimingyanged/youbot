//***************************************
//    Approximate Out-Of-Core KMeans    *
//            Mark Cummins              *
//     Oxford Mobile Robotics Group     *
//               2008/3/30              *
//***************************************

#include "pKMeans.h"
#include <map>
#include <iomanip>

void KMeans::InitializeClusterCentres()
{
    //Make sure CentresContainer is empty
    Centres.clear();
    Centres.reserve(m_nNumClus);

    //Two options, random and radius-based pre-cluster
    if(m_bRandomInitialization)
    {
        //Sets the cluster centres to randomly chosen datapoints
        //Biased towards denser clusters
        RandomInitialization();
    }
    else
    {
        //Sets the initial clusters based on an incremental radius based clustering
        // This is very expensive, but does seem to generate slightly better vocabularies for some purposes
        // See comparison in mark Cummins' thesis.
        RadiusBasedInitialization();

        //For large K (>100,000) and very many points (>10,000,000), this alternative version
        //will be faster:
        //RadiusBasedInitializationWithKDTree();
    }
    //Another option might be random points over the hypersphere that SURFs lie on.

    //Initialize the assignment of each point to a value out of range
    Assignment.reserve(m_nNumPts);
    Assignment.resize(0);
    for(unsigned int i=0;i<m_nNumPts;i++)
    {
        Assignment.push_back(m_nNumClus); //Value out of range
    }   

    double dfTimeTaken = HPMOOSTime() - m_dfStartTime;
    cout << "Initialization complete, in "  << setprecision(3) << ((dfTimeTaken - fmod(dfTimeTaken,60.0))/60.0) << " minutes " 
    << fmod(dfTimeTaken,60.0) << " seconds." << endl;
    cout << setprecision(7);
}

void KMeans::RandomInitialization()
{
    //Pick a random subset of m_NumClus points from the dataset.
    vector<unsigned int> random_subset_of_point_IDs;
    RandomKSubset(m_nNumClus,m_nNumPts,random_subset_of_point_IDs);

    const PointContainer& CPoints = Points; //Access points via a const reference. When using STXXL, this ensures that read buffers are not marked as dirty, so don't need to be written back to disk

    for(unsigned int i=0;i<m_nNumClus;i++)
    {
        ClusterCentre c(m_nDim);
        unsigned int point_index = m_nDim*random_subset_of_point_IDs[i];
        for(unsigned int j=0; j<m_nDim; j++,point_index++)
        {
            c.pos[j] = CPoints[point_index];
        }
        Centres.push_back(c);
    }
    m_nNumClustersSuggestedByInitializationMethod = Centres.size();
}

void KMeans::RandomKSubset(const unsigned int k, const unsigned int n, vector<unsigned int> &result)
{
    //Returns a sorted random subset of size k from the range [0..n), in O(k) time
    //(make sure n<2^32 - otherwise unsigned int overflows)
    //The algorithm used is a simple modification of the Knuth shuffle
    //See http://www.techuser.net/randpermgen2.html for discussion
    //Also perhaps useful in related contexts is Conveyor Belt sampling, see:
    //http://www.maths.abdn.ac.uk/~igc/tch/mx4002/notes/node86.html

    if(result.size() != 0)
        cout << "ERROR: RandomKSubset() called with non-empty result container." << endl;
    result.reserve(k);

    if(k<=n)
    {
        boost::random_number_generator<boost::lagged_fibonacci607> random_less_than(rng); //Random numbers in range [0,n)
        std::map<unsigned int,unsigned int> range;

        for(unsigned int i=0;i<k;i++)
        {
            unsigned int r = i + random_less_than(n-i); //Random number in range [i,n)
            //Swap(i,r)
            if(range.find(i) == range.end())
                range.insert(make_pair(i,i));
            if(range.find(r) == range.end())
                range.insert(make_pair(r,r));
            //Do swap
            unsigned int temp = range[r];
            range[r] = range[i];
            range[i] = temp;
        }

        for(unsigned int i=0;i<k;i++)
        {
            if(range.find(i) == range.end())
                cout << "ERROR: Looks like there is a bug in RandomKSubset()" << endl;

            result.push_back(range[i]);
        }
    }
    else
    {
        cout << "ERROR: RandomKSubset called with k>n" << endl;
    }
    std::sort(result.begin(),result.end());
}

void KMeans::RadiusBasedInitialization()
{
    //Use the old radius-based clustering to choose initial cluster centres for K-Means
    //The quality of this is OK, if you get the radius right
    //Avoids the bais towards denser clusters you get with random initialization.
    //For SURF vectors:
    //  radius of 0.5 seems about right (gives on the order of 200,000 words for 30k images)
    //            0.66 gives 10k words (probably too small, certainly for larger environments)

    //So, we'll pick a point, find it's nearest cluster centre
    //and if the nearest cluster centre is further than the threshold, declare the point to be a new cluster centre
    //otherwise, update the cluster to be the mean of it's members

    //If we end up with less than K clusters, we'll pick the balance randomly
    //If we end up with more than K, we'll pick a random K-subset

    //Const reference to the points container. When using STXXL, this ensures that buffers are not marked as dirty after a read, so they don't need to be written back to disk.
    const PointContainer& CPoints = Points; 

    //Set the first cluster centre to be at a random seed point
    boost::variate_generator<boost::lagged_fibonacci607&, boost::uniform_int<> > get_random_point_ID(rng,boost::uniform_int<>(0,m_nNumPts-1));
    unsigned int nSeedPointID = get_random_point_ID();

    vector<float> CurrentPoint(m_nDim);
    for(unsigned int i=0,j=m_nDim*nSeedPointID;i<m_nDim;i++,j++)
        CurrentPoint[i] = CPoints[j];
    Centres.push_back(ClusterCentre(CurrentPoint));
    unsigned int nCurrentNumClusters = Centres.size();

    int nNearestClusterID;
    const float fInitializationRadiusSq = m_fInitializationRadius*m_fInitializationRadius;

    cout << "Initializaing cluster centres using radius-based method." << endl;
    cout << "Points considered: ";

    //Consider the points from first to last
    //We could get more randomness by shuffling the points vector
    //However, if Points is large, this could be expensive
    for(unsigned int pointID=1, vectorOffset=1*m_nDim; pointID<m_nNumPts; ++pointID, vectorOffset+=m_nDim)
    {
        //*******************************************************
        // Find the point's nearest cluster centre
        // If this further than the threshold, start a new cluster
        // Otherwise, update the relevant cluster mean
        //*******************************************************

        //Construct the query
        //It isn't safe to pass about &(CPoints[j]) directly, becuase CPoints may be partially on disk.
        for(unsigned int i=0,j=vectorOffset;i<m_nDim;i++,j++)
        {
            CurrentPoint[i] = CPoints[j];
        }

        //Find the nearest cluster centre
        nNearestClusterID = -1;
        float fMinDistSq = std::numeric_limits<float>::infinity();
        float fDistSq;
        for(unsigned int clusID=0;clusID<nCurrentNumClusters;clusID++)
        {
            //Tried using a bail-out in this distance test (stop if distance is already greater than best know)
            //however, it actually made things a bit slower.
            fDistSq = FloatDistSquared(&(CurrentPoint[0]),&(Centres[clusID].pos[0]),m_nDim);
            if(fDistSq<fMinDistSq)
            {
                fMinDistSq = fDistSq;
                nNearestClusterID = clusID;
            }
        }

        if(fMinDistSq > fInitializationRadiusSq)
        {
            //If it's beyond the threshold, start a new cluster
            Centres.push_back(ClusterCentre(CurrentPoint));
            nCurrentNumClusters++;
        }
        else
        {
            //Adjust the existing centre
            //We have no notion of a point leaving a cluster here
            //A centre is simply at the centroid of all points it has ever "seen"
            //Obviously this is not great, but this is only an initialization method
            AddPointToCluster(nNearestClusterID,CurrentPoint);

            //Also, update centroid
            double dfNumPtsInCluster = (double) Centres[nNearestClusterID].num_points;
            for(unsigned int j=0;j<m_nDim;j++)
            {
                Centres[nNearestClusterID].pos[j] = (float)(Centres[nNearestClusterID].sum_of_point_positions[j]/dfNumPtsInCluster);
            }
        }

        //Statistics
        if(pointID % 1000 == 0)
            cout << pointID << "...";
    }

    cout << "Done" << endl; 

    //Now, add or subtract cluster centres to yield K
    cout << "Found "  << Centres.size() << " cluster centres" << endl;
    m_nNumClustersSuggestedByInitializationMethod = Centres.size();

    
    if(Centres.size() != m_nNumClus)
    {
        if(Centres.size() < m_nNumClus)
        {
            cout << "Adding some random centres to make it up to " << m_nNumClus << endl; 

            unsigned int nExtraNeeded = m_nNumClus-Centres.size();

            vector<unsigned int> random_subset_of_point_IDs;
            RandomKSubset(nExtraNeeded,m_nNumPts,random_subset_of_point_IDs);

            for(unsigned int i=0;i<nExtraNeeded;i++)
            {
                ClusterCentre c(m_nDim);
                unsigned int point_index = m_nDim*random_subset_of_point_IDs[i];
                for(unsigned int j=0; j<m_nDim; j++,point_index++)
                {
                    c.pos[j] = CPoints[point_index];
                }
                Centres.push_back(c);
            }
        }
        else
        {
            cout << "Returning a random K-subset of size " << m_nNumClus << endl; 

            vector<unsigned int> random_subset_of_centre_IDs;
            RandomKSubset(m_nNumClus,Centres.size(),random_subset_of_centre_IDs);

            for(unsigned int i=0;i<m_nNumClus;i++)
            {
                Centres[i] = Centres[random_subset_of_centre_IDs[i]];
            }
            Centres.resize(m_nNumClus);
        }
    }
    //Remove the num_points and sum_of_point_positions fields from the clusters.
    //After initialization, they should be just positions, with no membership.
    for(unsigned int i=0;i<m_nNumClus;i++)
    {
        Centres[i].num_points = 0;
        Centres[i].sum_of_point_positions.clear();
        Centres[i].sum_of_point_positions.resize(m_nDim,0.0);
    }
    
}


//So, this function does the same task as RadiusBasedInitialization
//but uses a kd-tree for nearest neighbour queries.
//Building a kd-tree each iteration is clearly going to be slower,
//both asymtotically and in terms of constants.
//The only way to maybe get a speed-up is to not to update kd-tree unless a cluster centre 
//moves by more than some threshold (making it very approximate indeed)
//After testing, it still seems that the simple approach is much faster
//Here, adding a cluster costs O(KlogK), above it costs O(1)
//Here, distance calculations are O(logK), above they are O(K)
//It seems that when K is large, and kd-tree rebuilds are not frequent, this version should be faster, but the cross-over point is only for very large K and many many points.
void KMeans::RadiusBasedInitializationWithKDTree()
{
    //Const reference to the points container. When using STXXL, this ensures that buffers are not marked as dirty after a read, so they don't need to be written back to disk.
    const PointContainer& CPoints = Points; 

    //Set the first cluster centre to be at a random seed point
    boost::variate_generator<boost::lagged_fibonacci607&, boost::uniform_int<> > get_random_point_ID(rng,boost::uniform_int<>(0,m_nNumPts-1));
    unsigned int nSeedPointID = get_random_point_ID();

    vector<float> CurrentPoint(m_nDim);
    for(unsigned int i=0,j=m_nDim*nSeedPointID;i<m_nDim;i++,j++)
        CurrentPoint[i] = CPoints[j];
    Centres.push_back(ClusterCentre(CurrentPoint));
    unsigned int nCurrentNumClusters = Centres.size();

    //Keep track of how far cluster centres have drifted
    //since last kd-tree build.
    CentresContainer CentresAtLastkdTreeRebuild;
    CentresAtLastkdTreeRebuild = Centres;

    //Initialize kd-Tree datastructures
    vector<float*> PointersToCentres;
    PointersToCentres.push_back(&(Centres[0].pos[0]));
    Index kdtree;
    int nNearestClusterID;

    const float fInitializationRadiusSq = m_fInitializationRadius*m_fInitializationRadius;

    //Magic number
    const float fMaximumClusterMovementSqBeforeKDTreeRebuild = fInitializationRadiusSq/25.0f;

    cout << "Initializaing cluster centres using radius-based method." << endl;
    cout << "Points considered: ";

    //Consider the points from first to last
    //We could get more randomness by shuffling the points vector
    //However, if Points is large, this could be expensive
    kdtree = BuildIndexFloat(&(PointersToCentres[0]),(int) nCurrentNumClusters,(int) m_nDim,m_nNumKDTrees);

    //KD tree build is about 250x slower than a single brute force search
    //So, we'll start off with brute force search, and only switch to brute force search if the
    //expected time between kd tree rebuilds is greater than 250 iterations
    bool bUseKDTree = false;
    const unsigned int ThresholdQueriesBetweenClusterAdds = 1000;//800;
    const unsigned int ConsecutiveThreshold = 10;    //How many times in a row do we need to see the query ratio exceeded before switching from brute force to kd-tree mode?
    unsigned int nNumPointsAddedSinceLastNewClusterCreated = 0;
    unsigned int nNumConsecutiveTimesQueryRatioAboveThreshold = 0;

    double dfTimerStart = HPMOOSTime();

    for(unsigned int pointID=1, vectorOffset=1*m_nDim; pointID<m_nNumPts; ++pointID, vectorOffset+=m_nDim)
    {
        //*******************************************************
        // Find the point's nearest cluster centre
        // If this further than the threshold, start a new cluster
        // Otherwise, update the relevant cluster mean
        //*******************************************************

        //Construct the query
        //We can't just pass &(CPoints[vectorOffset]) to the kd tree directly, becuase I think pointer-based access is not safe with STXXL. Data may be on disk. Need to access via the [] operator. 
        for(unsigned int i=0,j=vectorOffset;i<m_nDim;i++,j++)
        {
            CurrentPoint[i] = CPoints[j];
        }

        if(bUseKDTree)
        {
            //Find the nearest cluster centre via a query to the kd-tree.
            FindNeighborsFloat(&(nNearestClusterID),
                1,                   //How many near neighbours to find - We just want one
                &(CurrentPoint[0]),
                kdtree,
                m_nMaxKDTreeNodesToCheck);
        }
        else
        {
            //Or the brute force way
            nNearestClusterID = -1;
            float fMinDistSq = std::numeric_limits<float>::infinity();
            float fDistSq;
            for(unsigned int clusID=0;clusID<nCurrentNumClusters;clusID++)
            {
                //Tried using a bail-out in this distance test (stop if distance is already greater than best know)
                //however, it actually made things a bit slower, presumably becuase it interfered with loop unrolling
                fDistSq = FloatDistSquared(&(CurrentPoint[0]),&(Centres[clusID].pos[0]),m_nDim);
                if(fDistSq<fMinDistSq)
                {
                    fMinDistSq = fDistSq;
                    nNearestClusterID = clusID;
                }
            }
        }

        if(FloatDistSquared(&(CurrentPoint[0]),&(Centres[nNearestClusterID].pos[0]),m_nDim) > fInitializationRadiusSq)
        {
            //It's beyond the threshold, start a new cluster

            //Bookkeeping for mode switching
            if(!bUseKDTree)
            {
                 if(nNumPointsAddedSinceLastNewClusterCreated >= ThresholdQueriesBetweenClusterAdds)
                {
                    nNumConsecutiveTimesQueryRatioAboveThreshold++;
                    if(nNumConsecutiveTimesQueryRatioAboveThreshold>=ConsecutiveThreshold)
                    {
                        bUseKDTree = true;
                        cout << endl << "===========================" << endl;
                        cout << "Switching to using kd-tree!" << endl;
                        cout << "===========================" << endl;
                    }
                }
                else
                {
                    nNumConsecutiveTimesQueryRatioAboveThreshold = 0;
                }
                nNumPointsAddedSinceLastNewClusterCreated = 0;
            }

            //cout << "Starting new cluster with point " << pointID << ". Clusters: " << nCurrentNumClusters << endl;
            //cout << "  DistanceSq was " << FloatDistSquared(&(CurrentPoint[0]),&(Centres[nNearestClusterID].pos[0]),m_nDim) << endl;
            
            Centres.push_back(ClusterCentre(CurrentPoint));
            nCurrentNumClusters++;

            if(bUseKDTree)
            {
                cout << "Building a new kdTree at " << pointID << endl;
                //Refresh PointersToCentres
                PointersToCentres.clear();  //We can't just do this incrementally, becuase the Centres vector may have been reassigned, so the old pointers get invalidated.
                for(unsigned int i=0;i<nCurrentNumClusters;i++)
                {
                    PointersToCentres.push_back(&(Centres[i].pos[0]));
                }

                //Rebuild kdtree
                FreeIndex(kdtree);
                kdtree = BuildIndexFloat(&(PointersToCentres[0]),(int) nCurrentNumClusters,(int) m_nDim,m_nNumKDTrees);
                CentresAtLastkdTreeRebuild = Centres;
            }

        }
        else
        {
            nNumPointsAddedSinceLastNewClusterCreated++;
            //Adjust the existing centre
            //We have no notion of a point leaving a cluster here
            //A centre is simply at the centroid of all points it has ever "seen"
            //Obviously this is not great, but this is only an initialization method
            AddPointToCluster(nNearestClusterID,CurrentPoint);

            //Also, update centroid
            double dfNumPtsInCluster = (double) Centres[nNearestClusterID].num_points;
            for(unsigned int j=0;j<m_nDim;j++)
            {
                Centres[nNearestClusterID].pos[j] = (float)(Centres[nNearestClusterID].sum_of_point_positions[j]/dfNumPtsInCluster);
            }

            if(bUseKDTree && FloatDistSquared(&(Centres[nNearestClusterID].pos[0]),&(CentresAtLastkdTreeRebuild[nNearestClusterID].pos[0]),m_nDim)>fMaximumClusterMovementSqBeforeKDTreeRebuild)
            {
                //Cluster centre moved too far from position when kd-tree was last built
                //so go rebuild the kd-tree
                cout << "Refreshing a kdTree at " << pointID << endl;

                //Refresh PointersToCentres
                PointersToCentres.clear();  //We can't just do this incrementally, becuase the Centres vector may have been reassigned, so the old pointers get invalidated.
                for(unsigned int i=0;i<nCurrentNumClusters;i++)
                {
                    PointersToCentres.push_back(&(Centres[i].pos[0]));
                }

                //Rebuild kdtree
                FreeIndex(kdtree);
                kdtree = BuildIndexFloat(&(PointersToCentres[0]),(int) nCurrentNumClusters,(int) m_nDim,m_nNumKDTrees);
                CentresAtLastkdTreeRebuild = Centres;
            }
        }

        //Statistics
        if(pointID % 1000 == 0)
        {
            double dfTimerEnd = HPMOOSTime();
            //cout << pointID << "...";
            cout << "Points: " << pointID << " Clusters: " << Centres.size() << " Time: " << dfTimerEnd - dfTimerStart << endl;
            dfTimerStart = HPMOOSTime();
        }
    }

    //Tidy up
    FreeIndex(kdtree);

    cout << "Done" << endl; 

    
    //Now, add or subtract cluster centres to yield K
    cout << "Found "  << Centres.size() << " cluster centres" << endl;
    m_nNumClustersSuggestedByInitializationMethod = Centres.size();
    
    if(Centres.size() != m_nNumClus)
    {
        if(Centres.size() < m_nNumClus)
        {
            cout << "Adding some random centres to make it up to " << m_nNumClus << endl; 

            unsigned int nExtraNeeded = m_nNumClus-Centres.size();

            vector<unsigned int> random_subset_of_point_IDs;
            RandomKSubset(nExtraNeeded,m_nNumPts,random_subset_of_point_IDs);

            for(unsigned int i=0;i<nExtraNeeded;i++)
            {
                ClusterCentre c(m_nDim);
                unsigned int point_index = m_nDim*random_subset_of_point_IDs[i];
                for(unsigned int j=0; j<m_nDim; j++,point_index++)
                {
                    c.pos[j] = CPoints[point_index];
                }
                Centres.push_back(c);
            }
        }
        else
        {
            cout << "Returning a random K-subset of size " << m_nNumClus << endl; 

            vector<unsigned int> random_subset_of_centre_IDs;
            RandomKSubset(m_nNumClus,Centres.size(),random_subset_of_centre_IDs);

            for(unsigned int i=0;i<m_nNumClus;i++)
            {
                Centres[i] = Centres[random_subset_of_centre_IDs[i]];
            }
            Centres.resize(m_nNumClus);
        }
    }

    //Remove the num_points and sum_of_point_positions fields from the clusters.
    //After initialization, they should be just positions, with no membership.
    for(unsigned int i=0;i<m_nNumClus;i++)
    {
        Centres[i].num_points = 0;
        Centres[i].sum_of_point_positions.clear();
        Centres[i].sum_of_point_positions.resize(m_nDim,0.0);
    }
}
