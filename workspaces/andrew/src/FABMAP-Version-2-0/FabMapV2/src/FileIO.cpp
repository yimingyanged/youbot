//Functions to handle file output
#include "FabMap.h"
#include <iomanip>
#include "Sampler.h"
#include "RankingFunction.h"
#include "KeyframeDetector.h"

void FabMapCalculator::WriteResultsToDisk(string datasetName, double dfTimeTaken)
{
    if(!m_bTextOutputFormat)
    {
        //If we're outputing a Matlab file, record it now.
        //If output is txt, we've been recording as we go along.
        m_full_output_path = CreateOutputDirectory(EnsurePathHasTrailingSlash(m_base_output_path));
        WriteToFile(psame, "psame", datasetName, m_full_output_path);

        #ifdef ALLOW_DATA_ASSOCIATION
            WriteToFile(m_SceneToPlace, "sceneToPlace", datasetName, m_full_output_path);
        #endif
    }
    RecordConfigParamsToFile(m_full_output_path,dfTimeTaken);
    mp_KeyframeDetector->WriteKeyframesToDisk(datasetName,m_full_output_path);
}

void FabMapCalculator::WriteLineOfResults(unsigned int nImageID)
{
    if(m_bSparseOutputFormat)
    {
        if(m_bTopKOutput)
            WriteLineOfResults_SparseTopK(nImageID);
        else
            WriteLineOfResults_Sparse(nImageID);
    }
    else
    {
        WriteLineOfResults_Dense();
    }
}

void FabMapCalculator::WriteLineOfResults_Dense()
{
    //The current contents of location_probability get written out to file
    unsigned int max = location_probability.size();
    for(unsigned int i=0; i<max; ++i)
    {
        m_results_file << location_probability[i] << " ";
    }
    m_results_file << endl;
    
    //If data association is turned on, also record imageToPlace
    #ifdef ALLOW_DATA_ASSOCIATION
        m_SceneToPlace_file << m_SceneToPlace.back() << " ";
    #endif
}

void FabMapCalculator::WriteLineOfResults_Sparse(unsigned int nImageID)
{
    //All entries of the current location_probability above some threshold
    //get written out to file in sparse format
    m_results_file << "PDF due to image " << nImageID << " :" << endl;
    m_results_file << "Milliseconds to calculate: " << 1000.0*m_dfLastPDFCalculationTime << endl;

    //First write out the indices
    unsigned int max = location_probability.size();
    vector<double> NonZeroValues;
    for(unsigned int i=0; i<max; ++i)
    {
        if(location_probability[i]>m_dfSparseRecordingThreshold)
        {
            m_results_file << i << " ";
            NonZeroValues.push_back(location_probability[i]);
        }
    }
    m_results_file << endl;

    //Then write out the corresponding values.
    max = NonZeroValues.size();
    for(unsigned int i=0; i<max; ++i)
    {
        m_results_file << NonZeroValues[i] << " ";
    }
    m_results_file << endl;
    
    //If data association is turned on, also record imageToPlace
    #ifdef ALLOW_DATA_ASSOCIATION
        m_SceneToPlace_file << m_SceneToPlace.back() << " ";
    #endif
}

void FabMapCalculator::WriteLineOfResults_SparseTopK(unsigned int nImageID)
{
    //The K largest entries of the current location_probability
    //get written out to file in sparse format
    m_results_file << "PDF due to image " << nImageID << " :" << endl;
    m_results_file << "Milliseconds to calculate: " << 1000.0*m_dfLastPDFCalculationTime << endl;

    const unsigned int K = 100;
    //Return the probabilities of the K most likely places
    //in the format PlaceID:prob
    //with PlaceID 0-based
    //If there are fewer than K places, return them all

    double dfReportingThreshold;
    if(location_probability.size() < K)
    {
        dfReportingThreshold = -1.0; //Less than K places, so we want to return the probability of all places.
    }
    else
    {
        //Find the probability of the K-th most likely place.
        vector<double> TopK(K);
        partial_sort_copy(location_probability.begin(),location_probability.end(),TopK.begin(),TopK.end(),greater<double>());
        dfReportingThreshold = TopK[K-1];
    }
    
    //Now, add to the output all places where prob is >= dfReportingThreshold, up to a maximum of K places.
    vector<double> NonZeroValues;
    unsigned int nReported = 0;
    unsigned int max = location_probability.size();
    for(unsigned int i=0;i<max;i++)
    {
        if(location_probability[i]>dfReportingThreshold)
        {
            //Add it to the output
            m_results_file << i << " ";
            NonZeroValues.push_back(location_probability[i]);
        }
        if(nReported>K)
            break;
    }
    m_results_file << endl;

    //Then write out the corresponding values.
    max = NonZeroValues.size();
    for(unsigned int i=0; i<max; ++i)
    {
        m_results_file << NonZeroValues[i] << " ";
    }
    m_results_file << endl;
    
    //If data association is turned on, also record imageToPlace
    #ifdef ALLOW_DATA_ASSOCIATION
        m_SceneToPlace_file << m_SceneToPlace.back() << " ";
    #endif
}

void FabMapCalculator::RecordConfigParamsToFile(string output_path)
{
    //Keep a record of what parameters we ran the algorithm with.
    std::ofstream params_file((output_path + "config.params").c_str());

    params_file << "Mode: " << mp_RankingFunction->DescribeRankingFunction() << endl;

    params_file << "p(observe | exists) = "            <<    p_observe_given_exists        << endl;
    params_file << "p(observe | !exists) = "            <<    p_z_1_given_e_0[0]        << endl;
    params_file << "p(at new place) = "                <<    p_at_new_place                << endl;
    params_file << "Likelihood Smoothing Factor: "    << m_df_likelihood_smoothing_factor     << endl;

    //Prior
    params_file << mp_PriorProvider->DescribePrior();

    //Sampler
    params_file << "Sampler: ";
    if(mp_Sampler->DescribeSampler() == "MEAN FIELD")
    {
        params_file << "(No Sampling. Mean Field Approximation.)" << endl;
    }
    else
    {
        params_file << mp_Sampler->DescribeSampler() << endl;
        params_file << "Minimum Number Of Samples Used: ";
        if(mp_Sampler->FiniteSampler() && (m_nMinNumSamples<m_nMaxSamples))
        {
            params_file << m_nMinNumSamples << endl;
        }
        else
        {
            params_file << m_nMaxSamples << endl;
        }
        if(m_sAltSamplePath.size()>1)
        {
            params_file << "Alternate sampling file: " << m_sAltSampleFile << endl;
        }
    }


    //Data Association
#ifdef ALLOW_DATA_ASSOCIATION
    params_file << "Data Association: ON" << endl;
    params_file << "Data Association Threshold: "    << m_df_likelihood_smoothing_factor     << endl;
#else
    params_file << "Data Association: OFF" << endl;
#endif

    //Keyframe detector
    params_file << "Keyframe Detector: " << mp_KeyframeDetector->DescribeKeyframeDetector() << endl;

    //Disallow local matches
    if(m_bDisallowLocalMatches)
        params_file << "Disallow Local Matches: " << m_nDisallowRegion << endl;
    else
        params_file << "Disallow Local Matches: OFF" << endl;

    //Number of images processed
    if(!m_bProcessAllScenes)
    {
        params_file << "Images Processed: First " << m_nMaxNumberOfScenesToProcess << endl;
    }
    else
    {
        params_file << "Images Processed: All" << endl;
    }

    //Blob response filter
    params_file << "Blob Response Filter: " << m_dfBlobResponseThreshold << endl;

    params_file.close();
}

void FabMapCalculator::RecordConfigParamsToFile(string output_path, double dfTimeTaken)
{
    //Record standard parameters
    RecordConfigParamsToFile(output_path);

    //Now add time taken
    std::ofstream params_file((output_path + "config.params").c_str(),ofstream::out | ofstream::app);
    if(dfTimeTaken > 60.0)
    {
        params_file << endl << "Finished. Total time " << setprecision(3) << ((dfTimeTaken - fmod(dfTimeTaken,60.0))/60.0) << " minutes " 
                                        << fmod(dfTimeTaken,60.0) << " seconds." << endl;
    }
    else
    {
        params_file << endl << "Finished. Total time " << dfTimeTaken  << " seconds." << endl;
    }
    params_file.close();
}

void FabMapCalculator::CreateOutputFileStructures()
{
    //Creates output directories and sets up output file streams
    if(m_base_output_path != "")
    {
        //What to call it?
        string datasetName;
        if(!m_MissionReader.GetConfigurationParam("DatasetName", datasetName))
        {
            datasetName = "NoName";
        }

        //Create directories
        m_full_output_path = CreateOutputDirectory(EnsurePathHasTrailingSlash(m_base_output_path));

        //Initialize output file
        string sFileExtension = m_bSparseOutputFormat ? ".tms" : ".tmd";
        m_results_file.open((m_full_output_path + datasetName + "_psame"+sFileExtension).c_str(),ios::out);
        if(!m_results_file.is_open())
        {
            MOOSTrace("Failed to open file for output");
        }
        m_results_file << setprecision(16);

        //Write file header, if there is one
        if(m_bSparseOutputFormat)
        {
            if(!m_bTopKOutput)
                m_results_file << "SparseRecordingThreshold:" << m_dfSparseRecordingThreshold << endl;
            else
                m_results_file << "SparseRecordingThreshold: TopK" << endl;
        }

    #ifdef ALLOW_DATA_ASSOCIATION
        m_SceneToPlace_file.open((m_full_output_path + datasetName + "_sceneToPlace"+".txt").c_str(),ios::out);
        if(!m_SceneToPlace_file.is_open())
        {
            MOOSTrace("Failed to open file for sceneToPlace output");
        }
    #endif
    }
}
