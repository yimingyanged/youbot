#include "FabMap.h"
#include "Sampler.h"
#include "KeyframeDetector.h"
#include "RankingFunction.h"
#include <functional>
#include <iomanip>

// Read the Config.moos configuration file
// and setup a new FabMap object appropriately.
void FabMapCalculator::DoInitialSetup()
{    
    //****************************************
    //    Input parsing/filtering options     
    //****************************************
    m_dfBlobResponseThreshold = 0.0; 
    m_MissionReader.GetConfigurationParam("BlobResponseFilter", m_dfBlobResponseThreshold);

    //****************************************
    //    Initialize Location Prior Provider     
    //****************************************
    string Prior_type;
    m_MissionReader.GetConfigurationParam("PriorModel", Prior_type);
    MOOSToUpper(Prior_type);

    if(Prior_type == "UNIFORM_LOCATION_PRIOR")
    {
        mp_PriorProvider = new UniformLocationPrior(p_at_new_place);
    }
    else if(Prior_type == "ALWAYS_MOVE_MOTION_MODEL")
    {
        double dfForwardMotionBias;
        if(m_MissionReader.GetConfigurationParam("ForwardMotionBias", dfForwardMotionBias))
        {
            mp_PriorProvider = new LeftRightMotionModelPrior(p_at_new_place,dfForwardMotionBias);
        }
        else
        {
            mp_PriorProvider = new LeftRightMotionModelPrior(p_at_new_place);
        }
    }
    else if(Prior_type == "LEFT_RIGHT_BELOW_MOTION_MODEL")
    {
        mp_PriorProvider = new LeftRightBelowMotionModelPrior(p_at_new_place);
    }
    else
    {
        cout << "ERROR. Unrecognised Location Prior model:" << endl << Prior_type << endl << "Crash soon..." << endl;
    }

    //Optionally enforce a zero prior on the last N places.
    m_bDisallowLocalMatches = false;
    m_nDisallowRegion = 0;
    if(m_MissionReader.GetConfigurationParam("DisallowLocalMatches", m_nDisallowRegion))
        m_bDisallowLocalMatches = true;

    //****************************************
    //    Choose Ranking Function
    //****************************************
    string RankingFuction_type;
    m_MissionReader.GetConfigurationParam("RankingFunction", RankingFuction_type);
    MOOSToUpper(RankingFuction_type);

    if(RankingFuction_type == "FABMAP_NAIVE_BAYES")
        mp_RankingFunction = new NaiveBayesLikelihood(*this);
    else if(RankingFuction_type == "FABMAP_CHOW_LIU")
        mp_RankingFunction = new ChowLiuLikelihood(*this);
    else
        cout << "ERROR. Unrecognised Ranking Function Type:" << endl << RankingFuction_type << endl << "Crash soon..." << endl;

    //****************************************
    //    Initialize Sampler
    //****************************************
    int nMinSamples = 3000; // Default.
    m_MissionReader.GetConfigurationParam("MinNumSamplesToTake", nMinSamples);
    m_nMinNumSamples = (unsigned int) nMinSamples;

    string Sampler_type;
    m_MissionReader.GetConfigurationParam("SamplerType", Sampler_type);
    MOOSToUpper(Sampler_type);

    bool bAltSampleSourceSpecified = false;
    if(Sampler_type == "REAL_SAMPLES")
    {
        //Check if an alternate sampling set is specified.
         m_MissionReader.GetConfigurationParam("RealSamplePath", m_sAltSamplePath);
         m_MissionReader.GetConfigurationParam("RealSampleFile", m_sAltSampleFile);
         bAltSampleSourceSpecified = !m_sAltSamplePath.empty();
         m_sAltSamplePath = EnsurePathHasTrailingSlash(m_sAltSamplePath);
    }

    if(Sampler_type == "REAL_SAMPLES")
    {
        RealSampler* rs = bAltSampleSourceSpecified ? new RealSampler(*this,m_sAltSamplePath,m_sAltSampleFile+".oxs") : new RealSampler(*this);
        m_nMaxSamples = rs->GetMaxUniqueSamples();
        mp_Sampler = rs;
    }
    else if(Sampler_type == "MEAN_FIELD")
    {
        MeanFieldApprox* mfs = new MeanFieldApprox(*this);
        m_nMaxSamples = mfs->GetMaxUniqueSamples();
        mp_Sampler = mfs;
    }
    else if(Sampler_type == "NULL_SAMPLER")
    {
        NullSampler* ns = new NullSampler(*this);
        m_nMaxSamples = ns->GetMaxUniqueSamples();
        mp_Sampler = ns;
    }
    else
    {
        cout << "ERROR. Unrecognised Sampler Type:" << endl << Sampler_type << endl << "Crash soon..." << endl;
    }

    //****************************************
    //    Choose Keyframe Detector
    //****************************************
    string KeyframeDetector_type;
    m_MissionReader.GetConfigurationParam("KeyframeDetector", KeyframeDetector_type);
    MOOSToUpper(KeyframeDetector_type);

    if(KeyframeDetector_type == "" || KeyframeDetector_type == "NONE")
        mp_KeyframeDetector = new NullKeyframeDetector();
    else if(KeyframeDetector_type == "FIXED_INCREMENT")
    {
        int nIncrement = 5;    //Default
        m_MissionReader.GetConfigurationParam("KeyframeDetectorParameter", nIncrement);
        mp_KeyframeDetector = new FixedIncrementKeyframeDetector(nIncrement,m_nPreAllocateLength);
    }
    else if(KeyframeDetector_type == "WORDS_IN_COMMON")
    {
        double dfPercentInCommon_Threshold = 0.18;   //Default
        m_MissionReader.GetConfigurationParam("KeyframeDetectorParameter", dfPercentInCommon_Threshold);
        mp_KeyframeDetector = new WordsInCommonKeyframeDetector(dfPercentInCommon_Threshold,m_nPreAllocateLength);
    }
    else if(KeyframeDetector_type == "FABMAP_LIKELIHOOD")
    {
        mp_KeyframeDetector = new FabMapKeyframeDetector(*this);
        if(m_nDisallowRegion!=0)
        {
            m_nDisallowRegion = 0;
            cout << "Note: FABMAP_LIKELIHOOD keyframes does not play well with DisallowLocalMatches. Setting DisallowLocalMatches to zero." << endl;
        }
    }
    else
        cout << "ERROR. Unrecognised Keyframe Detector Type:" << endl << KeyframeDetector_type << endl;



    //****************************************
    //        Calculation Parameters
    //****************************************

    //Data Assoication Threshold
    m_dfDataAssociationThreshold = 0.99; //default, if none specified.
    m_MissionReader.GetConfigurationParam("DataAssociationThreshold",m_dfDataAssociationThreshold);

    //Fractional processing of input?
    m_bProcessAllScenes = !(m_MissionReader.GetConfigurationParam("MaxNumberOfScenesToProcess",m_nMaxNumberOfScenesToProcess));

    //****************************************
    //           I/O Preferences
    //****************************************

    //Output path
    m_MissionReader.GetConfigurationParam("BaseOutputPath",m_base_output_path);

    //Output format
    string sOutputFormat;
    m_bTopKOutput = false;
    m_MissionReader.GetConfigurationParam("OutputFormat",sOutputFormat);
    if(MOOSStrCmp(sOutputFormat,"MATLAB"))
    {
        //Nothing to set up.
        //We'll write out a matrix when the calculation is over.
        m_bTextOutputFormat = false; 
        m_bSparseOutputFormat = false;
    }
    else if(MOOSStrCmp(sOutputFormat,"TXT_DENSE"))
    {
        //Settings for dense output
        m_bTextOutputFormat = true; 
        m_bSparseOutputFormat = false;
        CreateOutputFileStructures();
    }
    else if(MOOSStrCmp(sOutputFormat,"TXT_SPARSE"))
    {
        //Settings for sparse output
        m_bTextOutputFormat = true; 
        m_bSparseOutputFormat = true;
    
        //Decide a sparse recording threshold
        string sTopK;
        m_MissionReader.GetConfigurationParam("SparseRecordingThreshold",sTopK);
        if(MOOSStrCmp(sTopK,"TopK"))
        {
            m_bTopKOutput = true;
        }
        else if(!m_MissionReader.GetConfigurationParam("SparseRecordingThreshold",m_dfSparseRecordingThreshold))
        {
            //None specified. Use default
            m_dfSparseRecordingThreshold = 0.1;
            cout << "WARNING. Sparse output requested, but no SparseRecordingThreshold specified. Defaulting to " << m_dfSparseRecordingThreshold  << endl; 
        }
    
        CreateOutputFileStructures();
    }
    else
    {
        cout << "WARNING. Unrecognised output format request: " << sOutputFormat << ". Defaulting to TXT_DENSE." << endl; 
        m_bTextOutputFormat = true; 
        m_bSparseOutputFormat = false;
        CreateOutputFileStructures();
    }
}

//  Process a whole dataset from a scenes file.
//  This is what runs if call FabMap from the command line.
void FabMapCalculator::BatchProcess(string sAppName, string sMissionFile)
{    
    //Find the mission file
    m_MissionReader.SetAppName(sAppName);
    if(!m_MissionReader.SetFile(sMissionFile.c_str()))
    {
        cout << "Warning! Configuration file not found: " << sMissionFile << endl;
    }

    DoInitialSetup();

    if(m_base_output_path == "")
    {
        cout << endl << "WARNING: Batch mode called with no output directory specified. Computation will output NOTHING!" << endl << endl;
    }

    //Read in Batch-Mode-specific parameters from Mission file.
    string datasetPath, datasetName, excludedRegionsFilePath;

    m_MissionReader.GetConfigurationParam("DatasetPath", datasetPath);
    m_MissionReader.GetConfigurationParam("DatasetName", datasetName);
    m_MissionReader.GetConfigurationParam("ExcludedRegionsDefinitionFile", excludedRegionsFilePath);

    //Now, do the calculation and time it.
    double dfStartTime = HPMOOSTime();
  
    //Set up an iterator to read incrementally.
    mp_OXSIterator = new OXSIterator(datasetPath,datasetName + ".oxs",InterestPointIsInExcludedRegionCheck(excludedRegionsFilePath),false,m_dfBlobResponseThreshold);
    
    unsigned int nObservationVocabSize;
    ParseOXS_PeekDimensions(datasetPath, datasetName + ".oxs",m_nNumInputObservations,nObservationVocabSize);  // How many observations are there to process?
    if(nObservationVocabSize != m_nVocabSize)
    {
        cerr << endl << endl << "ERROR. Vocabulary size mismatch." << endl << "Vocab has size " << m_nVocabSize << " and observations have size " << nObservationVocabSize << endl << "Aborting." << endl;
        exit(0);           //This is so bad we should stop immediately.
    }
    if(m_nNumInputObservations == 0)
    {
        cout << "Failed to find scenes file at path:" << endl << datasetPath << datasetName << ".oxs" << endl;
    }
    else
    {
        cout << endl << "STARTING BATCH JOB. Found " << m_nNumInputObservations << " scenes to process. Vocab has " << m_nVocabSize << " words." << endl;

        //Optionally limit the number of scenes we're going to actually consider
        if(!m_bProcessAllScenes)
        {
            m_nNumInputObservations = m_nMaxNumberOfScenesToProcess < (int) m_nNumInputObservations ? m_nMaxNumberOfScenesToProcess : (int) m_nNumInputObservations;
            cout << "Fractional processing requested. Going to process " << m_nNumInputObservations << " scenes only." << endl;
        }

        //If we're outputting to a Matlab matrix, initialize it
        if(!m_bTextOutputFormat)
        {
            psame.set_size(m_nNumInputObservations,m_nNumInputObservations);
            psame.fill(0.0);
        }

        //Now, go do the main calculation.
        CalculatePSameMatrix();

        //Finish
        double timeTaken = HPMOOSTime() - dfStartTime;
        WriteResultsToDisk(datasetName,timeTaken);

        if(timeTaken > 60.0)
        {
            cout << endl << "Finished. Total time " << setprecision(3) << ((timeTaken - fmod(timeTaken,60.0))/60.0) << " minutes " 
                << fmod(timeTaken,60.0) << " seconds." << endl;
        }
        else
        {
            cout << endl << "Finished. Total time " << timeTaken  << " seconds." << endl;
        }
    }   
}
