#include "FabMap.h"
#include "Sampler.h"
#include "KeyframeDetector.h"
#include "RankingFunction.h"
#include <functional>
#include <iomanip>

//**********************************************
// Implementation of the external API.
// See FabMap.h for a description of the API.
//**********************************************

// After creating a new FabMap object, this function must be called for the object to be usable.
// Takes as input the "Config.moos" file with FabMap configuration parameters.
void FabMapCalculator::ConfigureForExternalCalls(const string &sMissionFile)
{
    //Find the config file
    m_MissionReader.SetAppName("pFabMapV2");
    if(!m_MissionReader.SetFile(sMissionFile.c_str()))
        cout << "Warning. Config File " << sMissionFile << " not found..." << endl;

    DoInitialSetup();
    m_nObservationsProcessed = 0;
    m_bLastMatchConfirmed = false;
    m_bDiscardInfoFromLastMatch = false;
}

// Takes a bag-of-words observation as input, returns a PDF over places in the map.
// PDF is due to the observation, and any position prior we may have from previous observations.
void FabMapCalculator::ProcessObservation(const Observation &raw_observation,
                                          LocationProbabilityContainer &ComputedLocationProbability)
{
    //Create scene record from raw list of words
    SceneRecord observation(raw_observation);

    //Deal with data association from last timestep
    //We're giving the external user control, so we only do data association if they call
    //ConfirmLastMatch() after getting the results from ProcessObservation()
    if(m_nObservationsProcessed != 0 && !m_bLastMatchConfirmed && !m_bDiscardInfoFromLastMatch)
    {
        //External user did not verify the last data association by calling ConfirmLastMatch().
        //So the last observation was not used to update an existing place appearance model.
        //Instead add it to the map as a new place here.
        AddSceneToIndex(m_LastObservation,m_nNumberOfExistingPlaces);
    }

    //Compute the PDF
    ProcessObservation_ComputePDF(observation);

    //Return the result
    ComputedLocationProbability = location_probability;

    //We postpone data association until next timestep - let the user confirm or reject.
    m_LastObservation = observation;
    m_bLastMatchConfirmed = false;
    m_bDiscardInfoFromLastMatch = false;
    m_nObservationsProcessed++;
}

//External user calls this to confirm that they have verified the last suggested loop closure
//If the user does not call this function before the next call to ProcessObservation(),
//we default to assigning the observation to a new place.
//See comments in FabMap.h for details.
void FabMapCalculator::ConfirmLastMatch()
{
    //User accepts the most likely suggested data association from the last timestep
    if(m_nObservationsProcessed > 1 && !m_bLastMatchConfirmed && !m_bDiscardInfoFromLastMatch)  //Make sure we haven't done this already
    {
        //first, which place was the best match?
        unsigned int ml_place = GetCurrentMostLikelyPlace();
        AddSceneToIndex(m_LastObservation,ml_place);
        m_bLastMatchConfirmed = true;
    }
}

// This function allows the user to externally override the loop closure suggested by FabMap.
// So for example you might geometrically verify the top five most likely places suggested
// by FabMap and choose to associate the observation with say the 3rd most likely place.
// If that place has ID 27, then call ConfirmLastMatch(27).
void FabMapCalculator::ConfirmLastMatch(const int nMLplace)
{
    if(m_nObservationsProcessed > 1 && !m_bLastMatchConfirmed && !m_bDiscardInfoFromLastMatch)  //Make sure we haven't done this already
    {
        AddSceneToIndex(m_LastObservation,nMLplace);
        m_bLastMatchConfirmed = true;
    }
}

//You may also choose to discard the last observation, neither creating a new
//place nor updating an existing place in the map. Normally FabMap never uses
//this option, however if it makes sense for your system it can be done by
//calling this function.
void FabMapCalculator::DiscardInfoFromLastMatch()
{
    m_bDiscardInfoFromLastMatch = true;
}

// Clear the map.
void FabMapCalculator::FabMap_Restart()
{
    cout << "Resetting..." << endl;
    m_WordToScenes = InvertedIndex(m_nVocabSize);
    m_SceneRecordsInMap.clear();
    m_NegativeBaseline.clear();
    m_SceneToPlace.clear();

    location_probability.clear();
    psame.clear();

    m_bLastMatchConfirmed = false;
    m_bDiscardInfoFromLastMatch = false;

    m_nObservationsProcessed = 0;
    m_nNumberOfExistingPlaces = 0;
}

// Convenience function to use FabMap as an image search engine, e.g. for object recognition.
// This function does a pure image "query".
// It computes the likelihood over places in the map, without prior, and doesn't update anything in the map.
void FabMapCalculator::ProcessObservation_ComputeLikelihoodOnly(const Observation &raw_observation,
                                                                LocationProbabilityContainer &ComputedLocationProbability)
{
    //Create scene record from raw list of words
    SceneRecord observation(raw_observation);

     //Compute the PDF
    location_probability.resize(m_nNumberOfExistingPlaces);
    UniformLocationPrior uniform_prior(p_at_new_place);
    CalculatePDFOverLocation(observation,uniform_prior.GetLocationPrior(location_probability));

    //Return the result
    ComputedLocationProbability = location_probability;
}

// Convenience wrapper for ProcessObservation_ComputeLikelihoodOnly.
// Returns the index of the most likely place directly.
unsigned int FabMapCalculator::ProcessObservation_ComputePlaceWithHighestLikelihood(const Observation &raw_observation)
{
    LocationProbabilityContainer ComputedLocationProbability;
    ProcessObservation_ComputeLikelihoodOnly(raw_observation,ComputedLocationProbability);

    LocationProbabilityContainer::iterator max_pos = max_element(ComputedLocationProbability.begin(),ComputedLocationProbability.end());
    return distance(ComputedLocationProbability.begin(),max_pos);
}

//Allow external user to directly create new places.
void FabMapCalculator::AddToObservationToMap(const Observation &raw_observation)
{
    //Create scene record from raw list of words
    SceneRecord observation(raw_observation);
    AddSceneToIndex(observation);
    location_probability.resize(m_nNumberOfExistingPlaces+1);
}


//*****************************
//  Private helper functions
//*****************************

void FabMapCalculator::ProcessObservation_ComputePDF(const SceneRecord &observation)
{
    if(m_nObservationsProcessed == 0)                   //This is the first observation we've received. Set up pdf.
    {
        location_probability.resize(1);                 //Initialise the pdf
        location_probability[0] = 1.0;                  //We are certain we are at the first place. No other options.
    }
    else
    {
        //Calculate the position prior
        vector<double> prior = mp_PriorProvider->GetLocationPrior(location_probability);

        // Adjust prior to suppress matches to the last N places if requested.
       if(m_bDisallowLocalMatches)
            prior = mp_PriorProvider->ExcludeLocalMatches(prior,m_nDisallowRegion);

        //Update the pdf over location, given the observation
        CalculatePDFOverLocation(observation,prior);
    }

    //Record the result to disk
    if(m_base_output_path != "")
        WriteLineOfResults(m_nObservationsProcessed);
}
