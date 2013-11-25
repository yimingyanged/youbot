#include "FabMap.h"
#include "Sampler.h"
#include "RankingFunction.h"
#include "KeyframeDetector.h"
#include "ProgressMeter.h"

FabMapCalculator::FabMapCalculator(
                        string i_vocab_path, string i_vocabName,
                        double i_p_observe_given_exists,
                        double i_p_observe_given_not_exists,
                        double i_p_at_new_place,
                        unsigned int i_vocab_size,
                        double i_df_likelihood_smoothing_factor)
                        :
                        m_sVocabPath(i_vocab_path),
                        m_sVocabName(i_vocabName),
                        m_nNumberOfExistingPlaces(0),
                        p_z_1_given_e_0(i_vocab_size,i_p_observe_given_not_exists),
                        p_z_1_given_e_1(i_vocab_size,i_p_observe_given_exists),
                        p_observe_given_exists(i_p_observe_given_exists),
                        p_not_observe_given_exists(1.0-i_p_observe_given_exists),
                        one_over_p_not_observe_given_exists(1.0/(1.0-i_p_observe_given_exists)),
                        p_at_new_place(i_p_at_new_place),
                        m_df_likelihood_smoothing_factor(i_df_likelihood_smoothing_factor),
                        m_nVocabSize(i_vocab_size),
                        m_WordToScenes(i_vocab_size),
                        K1(i_vocab_size),
                        mp_Sampler(NULL),mp_PriorProvider(NULL),mp_RankingFunction(NULL),
                        mp_KeyframeDetector(NULL),mp_OXSIterator(NULL),
                        m_bLastMatchConfirmed(false),
                        m_bDiscardInfoFromLastMatch(false),
                        m_bDisallowLocalMatches(false),
                        m_nMostLikelyPlaceLastIteration(0),
                        m_dfLastPDFCalculationTime(0.0)
{
        //Initialize the random number generator
        //Fixed seed for repeatable debugging.
        srand(424242);   //srand ( static_cast<unsigned int>(time(NULL)) );

        //Read in the vocabulary properties
        ParseMatFile(m_sVocabPath,m_sVocabName,"Marginals",Marginals);
        ParseMatFile(m_sVocabPath,m_sVocabName,"RelevantConditionals",RelevantConditionals);
        ParseMatFile(m_sVocabPath,m_sVocabName,"RelevantNotConditionals",RelevantNotConditionals);
        ParseMatFile(m_sVocabPath,m_sVocabName,"ChowTree",Parent);
        GenerateCachedValues();
        Children = ReverseTree(Parent);

        //Pre-allocate some arrays. This prevents memory fragmentation and makes
        //a massive difference to overall memory footprint (fragmentation can
        //easily increase memory footprint 10x on longer runs).
        m_nPreAllocateLength = 150000;   //If expecting more than 150k places, this number should be increased
        location_probability.reserve(m_nPreAllocateLength);
}
        
FabMapCalculator::~FabMapCalculator()
{
    delete mp_Sampler;
    delete mp_PriorProvider;
    delete mp_RankingFunction;
    delete mp_KeyframeDetector;
    delete mp_OXSIterator;
}

//In batch mode, this is the main function that does the book-keeping and calls the inner loop.
void FabMapCalculator::CalculatePSameMatrix()
{
    //Create the first place model from the first scene record
    SceneRecord firstObservation;
    GetNextObservation(firstObservation);
    AddSceneToIndex(firstObservation);
    location_probability.resize(1);
    location_probability[0] = 1.0;  //Initially, we are certain we are at place 0.

    //Record this first line of pdf to results
    if(m_bTextOutputFormat)
    {
        WriteLineOfResults(0);
    }
    else
    {
        psame(0,0) = 1.0;
    }

    //Initialize the keyframe detector
    mp_KeyframeDetector->IsKeyframe(firstObservation,
                                    mp_PriorProvider->GetLocationPrior(location_probability),
                                    1);
    unsigned int nNumAcceptedKeyframes = 0;

    ProgressMeter progress(m_nNumInputObservations);
    cout << "Calculating the loop closure probabilities...";

    //Now, run the loop
    SceneRecord this_scene;
    vector<double> prior;
    prior.reserve(m_nPreAllocateLength);
    for(unsigned int i = 1;i<m_nNumInputObservations;i++)
    {
        //Record the most likely place last time.
        m_nMostLikelyPlaceLastIteration = GetCurrentMostLikelyPlace();

        //Calculate the position prior
        {   //Wrapper block helps prevent memory fragmentation.
            prior = mp_PriorProvider->GetLocationPrior(location_probability);

            if(m_bDisallowLocalMatches)
                prior = mp_PriorProvider->ExcludeLocalMatches(prior,m_nDisallowRegion);

            //Unpack the observation
            GetNextObservation(this_scene);

            //Check it's a keyframe.
            //Note that this code increments the main loop counter i. A little hairy!
            //Also note that some keyframe detectors call CalculatePDFOverLocation,
            //so will change the contents of location_probability.
            while(!mp_KeyframeDetector->IsKeyframe(this_scene,prior,i+1) && i<m_nNumInputObservations)
            {
                i++;    //Skip this iteration of the loop
                if(i<m_nNumInputObservations)
                    GetNextObservation(this_scene); //Fetch the next observation
            }
            nNumAcceptedKeyframes++;
            //If we rejected all remaining scenes, we're done
            if(!(i<m_nNumInputObservations))
                break;

            //Update the pdf over location
            CalculatePDFOverLocation(this_scene,prior);
        }

        //Update the map, given our position estimate
        DecideDataAssociation_andUpdatePlaceModels(this_scene);

        // Updating the total number of places that we have processed.
        m_nObservationsProcessed++;

        //Record the results
        if(m_bTextOutputFormat)
        {
            WriteLineOfResults(nNumAcceptedKeyframes);
        }
        else
        {
            //Store in matrix (we'll write the psame matrix to disk when we're done)
            for(unsigned int k = 0; k<location_probability.size(); k++)
            {
                psame(nNumAcceptedKeyframes,k) = location_probability[k];
            }
        }

        //Let the user know about our progress
        if(i%50==0)
        {
            cout << i << "...";
        }
        progress.WriteProgressLinear(i);
    }
    cout << "Done" << endl;
}

void FabMapCalculator::GetNextObservation(SceneRecord &NextObservation)
{
    if(mp_OXSIterator->HasNext())
    {
        NextObservation = *(*mp_OXSIterator);   //Fetch the observation
        ++(*mp_OXSIterator);                    //Increment the iterator
    }
    else
        cerr << "GetNextObservation() called, but no observations remain!" << endl;
}


void FabMapCalculator::AddSceneToIndex(const SceneRecord &observation)
{
    //No nAssociatedPlaceID supplied
    //Default assign this scene to a new place
    AddSceneToIndex(observation,m_nNumberOfExistingPlaces);
}

void FabMapCalculator::AddSceneToIndex(const SceneRecord &observation, unsigned int nAssociatedPlaceID)
{
    //Given a scene record
    //Add to SceneRecords
    //And update inverted index

    //Sanity check the input
    if(nAssociatedPlaceID > m_nNumberOfExistingPlaces)
    {
        cout << "ERROR. Bad placeID supplied for scene." << endl << " Next available scene ID: " << (m_nNumberOfExistingPlaces-1)  << endl << " Requested Scene ID: " << nAssociatedPlaceID << endl;
        nAssociatedPlaceID = m_nNumberOfExistingPlaces;
    }

    //Add to scene records
    m_SceneRecordsInMap.push_back(observation);
    m_SceneToPlace.push_back(nAssociatedPlaceID);
    m_NegativeBaseline.push_back(GetNegativeBaselineForScene(observation)); //Cache the negative votes for the scene.

    //Add to inverted index
    const unsigned int nSceneID = m_SceneRecordsInMap.size()-1;
    const unsigned int nNumWordsObserved = observation.nNumWords;
    for(unsigned int i=0;i<nNumWordsObserved;i++)
    {
        InvertedIndexEntry WordSighting;
        WordSighting.nSceneID = nSceneID;
        WordSighting.nN = observation.WordData[i].nN;

        m_WordToScenes[observation.WordData[i].nID].push_back(WordSighting);
    }

    //Update number of existing places if necessary
    if(nAssociatedPlaceID == m_nNumberOfExistingPlaces)
    {
        //We're creating a new place. Bump the counter.
        m_nNumberOfExistingPlaces++;
    }
}

//#define ALLOW_DATA_ASSOCIATION
int FabMapCalculator::DecideDataAssociation() const
{
#ifndef ALLOW_DATA_ASSOCIATION
    return -1;
#else
    //Check if the data association condition is met
    //If a place matches the observation with confidence better than the
    //data association threshold, we return that place ID
    //Otherwise, we return -1.

    //Which place is the best match for the current observation, and how likely is it?
    unsigned int ml_place = GetCurrentMostLikelyPlace();
    double ml_place_prob = location_probability.at(ml_place);

    if(ml_place_prob >= m_dfDataAssociationThreshold)
    {
        if(ml_place == m_nNumberOfExistingPlaces)
        {
            //Most likely place is a new place.
            return -1;
        }
        else
        {
            //Most likely place is an existing place. Return it's index.
            return ml_place;
        }
    }
    else
    {
        //No place met the data association threshold. Return "new place"
        return -1;
    }
#endif
}

void FabMapCalculator::DecideDataAssociation_andUpdatePlaceModels(const SceneRecord &observations)
{
#ifndef ALLOW_DATA_ASSOCIATION
    // Blind data association tends to be unstable and is disabled in batch mode
    // by default. We simply create a new place every time. The best way to
    // experiment with data association is via the external API defined in
    // FabMap.h. A secondary check such as a geometric verification is
    // recommended to confirm a data association decision, as accepting a wrong
    // data association will rapidly lead to recognition failure.
    AddSceneToIndex(observations);
#else
    //Check if the data association condition is met
    //We either update an existing place model, or create a new one
    int ml_place = DecideDataAssociation();

    if(ml_place == -1)
    {
        //Add a new place
        //Either the observation did not meet confidence threshold to be associated
        //with any existing place, or the it did meet the condition and was associated with the new place
        AddSceneToIndex(observations);
        
    }
    else
    {
        //One of the existing places met the Data Assoc. condition
        //Update its place model
        AddSceneToIndex(observations,ml_place);
    }
#endif
}

void FabMapCalculator::CalculatePDFOverLocation(const SceneRecord &this_scene,
                                                const vector<double> &location_prior)
{
    //Inputs:
    // this_scene: The current observation
    // location_prior: The location prior.
    //After calling this function:
    // FabMapCalculator.location_probability will contain the updated pdf over position

    double dfStart = HPMOOSTime();

    //This vector will hold the pdf over scenes given the most recent observation.
    //NB - location_probability is distinct. A location is a collection of scenes.
    const unsigned int number_of_existing_scenes = m_SceneRecordsInMap.size();
    vector<double> m_scene_probability(number_of_existing_scenes+1);

    //**********************************
    //  Decide how many samples to draw
    //**********************************

    //Take max(nMinSamples,2*number_of_existing_places) samples, subject to the MaxSample limit
    unsigned int num_samples = (m_nMinNumSamples>2*number_of_existing_scenes) ? m_nMinNumSamples : 2 * number_of_existing_scenes;

    if(mp_Sampler->FiniteSampler() && (num_samples>m_nMaxSamples))
        num_samples = m_nMaxSamples; //The sampler can only return a finite number of unique samples. No point in taking more samples than this.

    //**************************
    //     Draw the samples
    //**************************
    mp_Sampler->DrawNSamples(num_samples);

    //******************************************
    //             INNER LOOP
    //  Calculate the observation likelihood
    //******************************************
    vnl_vector<double> log_p_obs_given_scene(number_of_existing_scenes+num_samples,0.0);

    SampleIterator	itBegin = mp_Sampler->begin();
    SampleIterator	itEnd = mp_Sampler->end();
    CalculateAppearanceLikelihoods(log_p_obs_given_scene,
                                   this_scene,
                                   number_of_existing_scenes,
                                   mp_Sampler->m_SamplesInvIndex,
                                   mp_Sampler->m_SamplesNegativeBaseline,
                                   itBegin,
                                   itEnd);

    //*************************************************
    // Translate pdf over scenes into pdf over places
    //*************************************************
    //A location is a collection of scenes
    //We assume they are independent
    //So that if placeA consists of scene1 and scene2
    //then p(placeA) = p(scene1) + p(scene2)

    //First, optionally set zero likelihood on the last N matches, to exclude images with overlap
    //We used to do this by setting a zero prior in that region, which was simple and did not require this little bit of ugliness.
    //However, due to numerical issues, by the time we convert out of logs, the likelihood can reach zero in places too. 
    //So, with the combination, it's possible to end up with a final pdf which is zero everywhere.
    //So, numerically safer to zero the likelihood here, though not as neat and pretty as just modifying the prior.
    if(m_bDisallowLocalMatches)
    {
        const double neg_infinity = -std::numeric_limits<double>::infinity();
        int imin = number_of_existing_scenes<m_nDisallowRegion ? 0 : (number_of_existing_scenes-m_nDisallowRegion);
        for(unsigned int i=imin;i<number_of_existing_scenes;i++)
            log_p_obs_given_scene(i) = neg_infinity;
    }

    //Resize location_probability to hold existing places, plus one entry for the new place
    location_probability.clear();
    location_probability.resize(m_nNumberOfExistingPlaces+1,0.0);

    double normalizer =  SumOfLogs(log_p_obs_given_scene);
    const double neg_infinity = -std::numeric_limits<double>::infinity();
    for(unsigned int t=0; t<number_of_existing_scenes; t++)
    {   //This is p(scene(t)). (computed as likelihood(scene(t))/sum(likelihood(all_scenes))
        if(log_p_obs_given_scene(t) != neg_infinity)    //This test is purely for speed. Calling exp on -infinity is 100x slower than on any other value, and has result 0. Much faster to test like this.
            location_probability[m_SceneToPlace[t]] += exp(log_p_obs_given_scene(t)-normalizer);
    }

    //********************************************************************
    // Convert the likelihoods out of logs, smooth, and multiply by prior
    //********************************************************************

    for(unsigned int t=0; t<m_nNumberOfExistingPlaces; t++)
    {
        //Apply smoothing.
        location_probability[t] = m_df_likelihood_smoothing_factor*location_probability[t]  + (1-m_df_likelihood_smoothing_factor)/(1+m_nNumberOfExistingPlaces);
        //Now add in the prior.
        location_probability[t] *= location_prior[t];
    }
    //Treat samples slightly differently.
    double new_place_prob = 0;
    const unsigned int nLastSampleIndex = log_p_obs_given_scene.size();
    for(unsigned int t=number_of_existing_scenes; t<nLastSampleIndex; t++)
    {
        new_place_prob += exp(log_p_obs_given_scene(t)-normalizer);
    }
    //Normalization by num_samples.
    //if(num_samples>0)
    //    new_place_prob /= ((double)num_samples);
    //Smoothing
    new_place_prob = m_df_likelihood_smoothing_factor*new_place_prob  + (1.0-m_df_likelihood_smoothing_factor)/(1.0+m_nNumberOfExistingPlaces);
    //Now add in the prior
    new_place_prob *= location_prior[m_nNumberOfExistingPlaces];
    location_probability[m_nNumberOfExistingPlaces] = new_place_prob;

    //Finally normalize
    double sum=0;
    for(unsigned int t=0;t<=m_nNumberOfExistingPlaces;t++)
    {
        sum += location_probability[t];
    }
    for(unsigned int t=0;t<=m_nNumberOfExistingPlaces;t++)
    {
        location_probability[t] /= sum;
    }

    // Record how long it took
    m_dfLastPDFCalculationTime = HPMOOSTime()-dfStart;
}

//******************************
//        Helper Functions
//******************************

//Takes as input a vnl_vector with entries [log(v1),log(v2),...,log(vn)] and returns log(v1+v2+...+vn).
inline double FabMapCalculator::SumOfLogs(const vnl_vector<double> &LogValues) const
{
    double sum = 0.0;
    //For better numerical precision, we'll shift largest value to zero, then add, the shift back.
    double shift = LogValues.max_value();    //a value between 0 and -Inf
    const int num_entries = LogValues.size();
    const double neg_infinity = -std::numeric_limits<double>::infinity();  //Manipulating infinity is very slow, so handle specially. Makes the function 100x faster if the vector is mostly infinity values.
    for(int t=0;t<num_entries;t++)
    { 
        if(LogValues(t) != neg_infinity)
            sum += exp(LogValues(t)-shift); //After the shift, the most likely place has value 0    
    }
    return log(sum) + shift;    //Shift back
}

void FabMapCalculator::GenerateCachedValues()
{
    cout << "Generating cached values...";
    //Pre-calculate and store some values used repeated by the coupled detector equations.
    for(unsigned int i =0; i<m_nVocabSize;i++)
    {
        K1[i] = (Marginals[i]*p_not_observe_given_exists)/((1.0-Marginals[i])*p_observe_given_exists);
    }
    cout << "Done" << endl;
}

vector<vector<int> > FabMapCalculator::ReverseTree(vnl_vector<int> parent)
{
    cout << "Building the child-pointer list...";
    //Given a vector specifying the parent of each node in the tree
    //return a vector of vectors, specifying the list of children of each node.
    vector<vector<int> > child_list(m_nVocabSize);

    for(unsigned int i=0;i<parent.size();++i)
    {
        if(parent[i] != -1)
            child_list[parent[i]].push_back(i);
    }

    cout << "Done" << endl;
    return child_list;
}

unsigned int FabMapCalculator::GetCurrentMostLikelyPlace() const
{
    LocationProbabilityContainer::const_iterator max_pos = max_element(location_probability.begin(),location_probability.end());
    return distance(location_probability.begin(),max_pos); 
}
