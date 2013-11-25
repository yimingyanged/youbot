#include "pKMeans.h"
#include "FileIO.h"
#include "mpi_kmeans.h"
#include "MOOSGenLib/ProcessConfigReader.h"
#include "IOHelperFunctions.h"
#include <iomanip>
#include <cmath>


void KMeans::Run()
{
    m_dfStartTime = HPMOOSTime();
    
    LoadInputData();

    if(m_nNumPts >0 && m_nNumClus > 0 && m_nNumClus < m_nNumPts)
    {
        InitializeClusterCentres();

        if(m_nNumRestarts == 0)
        {
            CallKMeans();
            cout << "...calculating final SSE...";
            m_fFinalSSE = CalculateSSE();
        }
        else
        {
            float dfBestSSE = std::numeric_limits<float>::infinity();
            CentresContainer BestCentres;
            AssignmentsConatiner BestAssignment;

            for(unsigned int i=0;i<=m_nNumRestarts;i++)
            {
                InitializeClusterCentres();
                CallKMeans();
                float dfSSEofRun = CalculateSSE();
                cout << "=======================" << endl
                    << "    Run " << (i+1) << ". SSE: " << dfSSEofRun;
                if(dfSSEofRun < dfBestSSE)
                {
                    dfBestSSE = dfSSEofRun;
                    BestCentres = Centres;
                    BestAssignment = Assignment;
                    cout << " (new best)";
                }
                cout << endl << "=======================" << endl;
            }
            //Put best values for writing to file
            Centres = BestCentres;
            Assignment = BestAssignment;
            m_fFinalSSE = dfBestSSE;
        }
    }
    else
    {
        cout << endl << "ERROR: Bad input. Points: " << m_nNumPts << ", Clusters requested: " << m_nNumClus << endl << endl;
    }

    WriteResults();

    //Finish
    cout << "Done." << endl;
    double timeTaken = HPMOOSTime() - m_dfStartTime;
    if(timeTaken > 60.0)
    {
        cout << endl << "Total time " << setprecision(3) << ((timeTaken - fmod(timeTaken,60.0))/60.0) << " minutes " 
            << fmod(timeTaken,60.0) << " seconds." << endl;
    }
    else
    {
        cout << endl << "Total time " << timeTaken  << " seconds." << endl;
    }
}

void KMeans::CallKMeans()
{
    if(m_bRunExactKMeans)
    {
#ifdef USE_STXXL_OUT_OF_CORE_CONTAINERS
        //Note: We have to do this because we pass data to the MPI library via &(Points[0])
        //which is not safe with external conatiners
        //One option would be to copy all data into an internal container and then pass to STXXL
        //but this would be slow and likely to run out of memory.
        //Might be more polite to try that though.
        //Just return error for the moment.
        cout << "Exact KMeans is not available with out of core conatiners enabled. Please compile without STXXL." << endl;
#else
        //Convert centres into a single long vector for MPI_KMeans library
        vector<float> CX;
        CX.reserve(m_nNumClus*m_nDim);
        for(unsigned int i=0;i<m_nNumClus;i++)
        {
            for(unsigned int j=0;j<m_nDim;j++)
            {
                CX.push_back(Centres[i].pos[j]);
            }
        }
     
        //Call lib
        kmeans(&(CX[0]),&(Points[0]),&(Assignment[0]),m_nDim,m_nNumPts,m_nNumClus,m_nMaxIterations,0,false);

        //Read the result back into our centres structure
        for(unsigned int i=0;i<m_nNumClus;i++)
        {
            for(unsigned int j=0;j<m_nDim;j++)
            {
               Centres[i].pos[j] = CX[j+i*m_nDim];
            }
        }
#endif
    }
    else
    {
        RunApproxOutOfCoreKMeans();
    }
}

void KMeans::ReadConfigurationParameters()
{
    //Object to read from MOOS mission files
    CProcessConfigReader MissionReader;
    MissionReader.SetFile(m_sMissionFile);
    MissionReader.SetAppName("pKMeans");

    //Input location
    m_sDataPath = "";
    MissionReader.GetConfigurationParam("InputPath", m_sDataPath);
    if(m_sDataPath=="")
        cout << "ERROR: No data directory supplied in configuration file!" << endl;
    m_sDataPath = EnsurePathHasTrailingSlash(m_sDataPath);

    m_sFileType = "";
    m_bReadFromSingleFile = false;
    MissionReader.GetConfigurationParam("FileType", m_sFileType);
    if(m_sFileType=="")
    {
        //Assume we're going to read only a single file.
        m_sFile = "";
        MissionReader.GetConfigurationParam("File", m_sFile);
        if(m_sFile == "")
            cout << "ERROR: Neither file nor file-type specified in configuration file. Nothing to cluster!" << endl;
        else
            m_bReadFromSingleFile = true;
    }

    m_sSubdirName = "";
    m_bReadFromDirectoryTree = MissionReader.GetConfigurationParam("SubdirectoryName", m_sSubdirName);
    if(m_sSubdirName!="" && m_sFileType=="")
        cout << "ERROR: Subdirectory parsing requested, but no file type specified. Nothing wil be read!" << endl;
    m_sSubdirName = EnsurePathHasTrailingSlash(m_sSubdirName);

    m_bFilterInputDataOnBlobResponse = MissionReader.GetConfigurationParam("BlobResponseFilter",m_dfBlobFilterThreshold);



    //Output options
    m_sOutputPath = m_sDataPath; //Default to placing output in the input path
    MissionReader.GetConfigurationParam("BaseOutputPath", m_sOutputPath);
    m_sOutputPath = EnsurePathHasTrailingSlash(m_sOutputPath);

    m_bPrintSSE = false;
    MissionReader.GetConfigurationParam("PrintSSE", m_bPrintSSE);

    m_bOXVOutputFormat = true;
    string sOutputFormat = "";
    MissionReader.GetConfigurationParam("OutputFormat", sOutputFormat);
    if(sOutputFormat != "")
        m_bOXVOutputFormat = (sOutputFormat == "OXV");


    m_bUniqueOutputSubdir = false;
    MissionReader.GetConfigurationParam("UniqueOutputSubdir", m_bUniqueOutputSubdir);

    m_bWriteAssignments = false;        //Do we want to output a points-to-clusters assignment file
    MissionReader.GetConfigurationParam("WriteAssignments", m_bWriteAssignments);

    //Which version of the algorithm?
    m_bRunExactKMeans = true;
    string sKmeansImplementation;
    MissionReader.GetConfigurationParam("KmeansImplementation", sKmeansImplementation);
    MOOSToUpper(sKmeansImplementation);
    m_bRunExactKMeans = (sKmeansImplementation == "EXACT");

    //clustering parameters
    m_nNumClus = 0;
    MissionReader.GetConfigurationParam("NumberOfClusters", m_nNumClus);
    if(m_nNumClus==0)
        cout << "ERROR: Read a request for zero clusters from configuration file!" << endl;

    m_nNumRestarts = 0;
    MissionReader.GetConfigurationParam("NumberOfRestarts", m_nNumRestarts);

    m_dfClusterMergingThreshold = 0.0;
    MissionReader.GetConfigurationParam("ClusterMergingThreshold", m_dfClusterMergingThreshold);
    m_bMergeCloseClusters = (m_dfClusterMergingThreshold != 0.0);


    //Termination Conditions
    m_nMaxIterations = 0;
    MissionReader.GetConfigurationParam("MaxIterations", m_nMaxIterations);

    m_dfMaxRunTime = std::numeric_limits<double>::infinity();
    MissionReader.GetConfigurationParam("MaxRunTimeInMinutes", m_dfMaxRunTime);
    m_dfMaxRunTime = 60.0*m_dfMaxRunTime;

    m_fSSEConvergenceThreshold = 0.0f;
    MissionReader.GetConfigurationParam("SSEChangeFactor", m_fSSEConvergenceThreshold);
    if(m_fSSEConvergenceThreshold < 0.0f || m_fSSEConvergenceThreshold > 1.0f)
        cout << "WARNING: SSEChangeFactor set to " << m_fSSEConvergenceThreshold <<  ". Should be a value between 0 and 1." << endl;
    m_bUseSSEbasedConvergenceTest = (m_fSSEConvergenceThreshold != 0.0f);
    m_bPrintSSE = m_bPrintSSE ||  m_bUseSSEbasedConvergenceTest;
  
    //Refinement options
    m_bDoRefinementStep = false;
    MissionReader.GetConfigurationParam("FinalRefinementStep", m_bDoRefinementStep);
    if(m_bDoRefinementStep && !m_bUseSSEbasedConvergenceTest)
        cout << "WARNING: Refinement step requested, but SSE convergence test not set. Refinement will never be called." << endl;

    m_nNodesToCheckMultiplier = 5;
    MissionReader.GetConfigurationParam("NodesToCheck_Multiplier", m_nNodesToCheckMultiplier);

    m_nRefinementCycles = 1;
    MissionReader.GetConfigurationParam("RefinementCycles", m_nRefinementCycles);

    //Initialization
    m_bRandomInitialization = true;
    string sInitializationMethod;
    MissionReader.GetConfigurationParam("InitializationMethod", sInitializationMethod);
    MOOSToUpper(sInitializationMethod);
    m_bRandomInitialization = (sInitializationMethod == "RANDOM");

    m_fInitializationRadius = -1.0f;
    if(!m_bRandomInitialization)
    {
        MissionReader.GetConfigurationParam("InitializationRadius", m_fInitializationRadius);
        if(m_fInitializationRadius<0.0f)
            cout << "ERROR: Read a request for zero InitializationRadius from configuration file!" << endl;
    }

    //kd-tree parameters
    m_nNumKDTrees = 8;
    m_nMaxKDTreeNodesToCheck = 100; 

    MissionReader.GetConfigurationParam("NumberOfkdTrees", m_nNumKDTrees);
    if(m_nNumKDTrees==0)
        cout << "ERROR: Read a request for zero kd-trees from configuration file!" << endl;

    MissionReader.GetConfigurationParam("KDTreeNodesToCheck", m_nMaxKDTreeNodesToCheck);
    if(m_nNumKDTrees==0)
        cout << "ERROR: KDTreeNodesToCheck was set to zero in the configuration file!" << endl;
}

void KMeans::LoadInputData()
{
    //Read the algorithm parameters from the configuration file
    ReadConfigurationParameters();

    //Read the data from disk
    if(m_bReadFromSingleFile)
    {
      string sFilename = m_sDataPath+m_sFile;
      KMeansIO::read_data_from_file(sFilename,Points,m_nDim,m_bFilterInputDataOnBlobResponse,m_dfBlobFilterThreshold);
    }
    else if(m_bReadFromDirectoryTree)
    {
        KMeansIO::read_data_from_directory_tree(m_sDataPath,m_sSubdirName,m_sFileType,Points,m_nDim,m_bFilterInputDataOnBlobResponse,m_dfBlobFilterThreshold);
    }
    else
        KMeansIO::read_data_from_directory(m_sDataPath,m_sFileType,Points,m_nDim,m_bFilterInputDataOnBlobResponse,m_dfBlobFilterThreshold);

    //Report
    m_nNumPts = Points.size()/m_nDim;                    
    cout << "Input (size " << Points.size() << ", i.e. " << m_nNumPts << " points):" << endl;
}

void KMeans::WriteResults()
{
    cout << "Writing to file...";

    string sOutputDir;
    if(m_bUniqueOutputSubdir)
        sOutputDir = CreateOutputDirectory(m_sOutputPath);
    else
        sOutputDir = m_sOutputPath;        
    
    //Write out assignments
    if(m_bWriteAssignments)
    {
        string sAssignmentsFile;
        if(m_bReadFromSingleFile)
            sAssignmentsFile = sOutputDir + m_sFile + "_assignments";
        else
            sAssignmentsFile = sOutputDir + "Assignments.txt";

        ofstream assignments_file(sAssignmentsFile.c_str());

        if( !assignments_file ) 
        {
            cerr << "ERROR saving assignments: " << endl
                << "Couldn't open file '" << sAssignmentsFile.c_str() << "'!" << endl;
        }

        for(unsigned int i=0;i<m_nNumPts;++i)
        {
            assignments_file << Assignment[i] << endl;
        }
        assignments_file.close();
    }

    //Write out cluster centres
    string sCentresFile;
    if(m_bOXVOutputFormat)
    {
        sCentresFile = sOutputDir + "Vocab.oxv";
    }
    else
    {
        if(m_bReadFromSingleFile)
            sCentresFile = sOutputDir + m_sFile + "_centres";
        else
            sCentresFile = sOutputDir + "Centres.txt";
    }
    
    ofstream centres_file(sCentresFile.c_str());

    if( !centres_file ) 
    {
        cerr << "ERROR saving cluster centres: " << endl
            << "Couldn't open file '" << sCentresFile.c_str() << "'!" << endl;
    }

    if(m_bOXVOutputFormat)
    {
        centres_file << "WORDS:" << m_nNumClus << endl;
        //CLUSTER_THRESHOLD isn't necessarily meaningful any more, but we'll write it anyway.
        centres_file << "CLUSTER_THRESHOLD:" << m_fInitializationRadius << endl;
    }

    for(unsigned int i=0;i<Centres.size();++i)
    {
        if(m_bOXVOutputFormat)
        {
            centres_file << "WORD:" << i << endl;
        }
        for(unsigned int j=0;j<m_nDim;j++)
        {    
            centres_file << Centres[i].pos[j] << " ";
        }
        centres_file << endl;
    }
    centres_file.close();

    RecordConfigParamsToFile(sOutputDir);
}

void KMeans::RecordConfigParamsToFile(string output_path)
{
    //Keep a record of what parameters we ran the alg with.
    std::ofstream params_file((output_path + "config.params").c_str());

    params_file << "Number of clusters: " << m_nNumClus << endl;
    params_file << "Number of points: " << m_nNumPts << endl;
    params_file << "Dimensionality: " << m_nDim << endl;
    if(m_bFilterInputDataOnBlobResponse)
        params_file << "Blob response filtered at: " << m_dfBlobFilterThreshold << endl;
    params_file << endl;

    params_file << "Input Data: " << m_sDataPath << endl;
    if(m_bRunExactKMeans)
        params_file << "Algorithm: EXACT" << endl;
    else
        params_file << "Algorithm: APPROX_OUT_OF_CORE" << endl;

    if(m_bRandomInitialization)
        params_file << "Initialization: RANDOM" << endl;
    else
        params_file << "Initialization: RADIUS_BASED with radius" << m_fInitializationRadius << endl;

    if(m_bMergeCloseClusters)
        params_file << "Merging: ON, with radius " << m_dfClusterMergingThreshold << endl;
    else
        params_file << "Merging: OFF " << endl;

    params_file << endl << "Termination Condition: ";
    if(m_nMaxIterations == 0 && m_dfMaxRunTime == std::numeric_limits<double>::infinity() && !m_bUseSSEbasedConvergenceTest)    
    {
        params_file << "CONVERGENCE" << endl;
    }
    else
    {
        if(m_nMaxIterations != 0) 
        {
            params_file << endl << " " <<  m_nMaxIterations << " iterations max. ";
        }
        if(m_dfMaxRunTime != std::numeric_limits<double>::infinity())
        {
            params_file << endl << " Run time " << m_dfMaxRunTime/60.0 << " minutes max. ";
        }
        if(m_bUseSSEbasedConvergenceTest)
        {
            params_file << endl << " SSE Convergence Threshold " << m_fSSEConvergenceThreshold;
            if(m_bDoRefinementStep)
            {
                params_file << " (using refinement. Cycles: " << m_nRefinementCycles << " Multiplier: " << m_nNodesToCheckMultiplier << ")";
            }
        }
        params_file << endl;
    }

    params_file << "Restarts: " << m_nNumRestarts << endl;
    params_file << "Number of kd-trees: " << m_nNumKDTrees << endl;
    params_file << "Max kd-tree nodes to check: " << m_nMaxKDTreeNodesToCheck << endl;

    params_file << endl;
    params_file << "Num clusters suggested by initialization method: " << m_nNumClustersSuggestedByInitializationMethod << endl;
    params_file << "Initial SSE: " << m_fInitialSSE << endl;
    params_file << "Final SSE: "   << m_fFinalSSE   << endl;
    params_file << "Iterations completed: "   << m_nIterationsCompleted   << endl;

    double dfTimeTaken = HPMOOSTime() - m_dfStartTime;
    if(dfTimeTaken > 60.0)
    {
        params_file << endl << "Total time " << setprecision(3) << ((dfTimeTaken - fmod(dfTimeTaken,60.0))/60.0) << " minutes " 
                                        << fmod(dfTimeTaken,60.0) << " seconds." << endl;
    }
    else
    {
        params_file << endl << "Total time " << dfTimeTaken  << " seconds." << endl;
    }

    params_file.close();
}

float KMeans::CalculateSSE()
{
    //Returns the sum of squared errors of a clustering

    const PointContainer& CPoints = Points; //When using STXXL, access via a const reference ensures that buffers are not marked as dirty after a read, so they don't need to be written back to disk.
    vector<float> CurrentPoint(m_nDim);
    unsigned int nNearestClusterID;

    float dfSSE = 0.0f;

    for(unsigned int pointID=0, vectorOffset=0; pointID<m_nNumPts; ++pointID, vectorOffset+=m_nDim)
    {
        nNearestClusterID = Assignment[pointID];

        //We can't just pass &(CPoints[vectorOffset]) to the kd tree directly, becuase I think pointer-based access is not safe with STXXL. Data may be on disk. Need to access via the [] operator. 
        for(unsigned int i=0,j=vectorOffset;i<m_nDim;i++,j++)
        {
            CurrentPoint[i] = CPoints[j];
        }

        //Get distance
        dfSSE += FloatDistSquared(&(CurrentPoint[0]),&(Centres[nNearestClusterID].pos[0]),m_nDim);
    }

    return dfSSE;
}
