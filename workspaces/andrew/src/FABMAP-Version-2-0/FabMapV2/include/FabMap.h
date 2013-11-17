#ifndef FABMAP_H
#define FABMAP_H 1

#include <string>
#include <limits>
#include <vector>
#include <deque>
#include <map>
#include <list>
#include <algorithm>

#include "config.h"
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include <MOOSLIB/MOOSLib.h>
#include "MOOSGenLib/MOOSGenLibGlobalHelper.h"

#include "OXS_parser.h"
#include "OXS_iterator.h"
#include "MAT_parser.h"
#include "IOHelperFunctions.h"
#include "LocationPrior.h"

#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/random.hpp>

#include "TypeDefinitions.h"

using namespace std;
using namespace boost::numeric;

//Forward Declarations
class Sampler;
class SampleIterator;
class RankingFunction;
class KeyframeDetector;

class FabMapCalculator  : public CMOOSApp
{
    //Declare samplers as friends
    friend class Sampler;
    friend class PlaceSampler;
    friend class WorldSampler;
    friend class TreeSampler;
    friend class MetaSampler;
    friend class RealSampler;
    //And also ranking functions
    friend class RankingFunction;
    friend class FabMapRanking;
    friend class NaiveBayesLikelihood;
    friend class ChowLiuLikelihood;
    //And for the FabMap keyframe detector
    friend class KeyframeDetector;
    friend class FabMapKeyframeDetector;

public:
    FabMapCalculator( string i_vocab_path, string i_vocabName,
                        double i_p_observe_given_exists,
						double i_p_observe_given_not_exists,
                        double i_p_at_new_place,
                        unsigned int i_vocab_size,
                        double i_df_likelihood_smoothing_factor);

    virtual ~FabMapCalculator();

    //*************************************************************************
    //                     Stand-alone mode
    //         Process a whole dataset from a scenes file.
    //  This function is called if you run FabMap from the command line.
    //*************************************************************************
    void BatchProcess(string sAppName, string sMissionFile);
    
    //**************************************************************************
    //                      External API
    // Call these functions to use FabMap as a library within another program.
    //**************************************************************************

    // Supply the "Config.moos" file with FabMap configuration parameters.
    // After creating a new FabMap object, this must be called for the object to be usable.
    void ConfigureForExternalCalls(const string &sMissionFile);

    // Takes a bag-of-words observation as input, returns a PDF over places in
    // the map.
    //
    // Note: The PDF returned combines evidence from this observation with the
    // location prior that has build up due to previous observations. Currently
    // the API has no method to allow the location prior to be specified
    // externally, but if you have a good external motion estimate then using
    // this to supply a location prior would be a good thing to add! The motion
    // model inside FabMap is very crude, basically just a proof of concept. It
    // You could supply your own prior with small changes to ExternalAPI.cpp
    void ProcessObservation(const Observation &raw_observation,
                            LocationProbabilityContainer &ComputedLocationProbability);

    // After calling ProcessObservation, the external user must call
    // ConfirmLastMatch() to accept the loop closure suggestion from FabMap. If
    // ConfirmLastMatch() is not called, FabMap will discard the loop closure
    // and instead add the last observation to the map as a new place.
    //
    // This is intended to allow the use of external information to filter
    // FabMap loop closure suggestions. For example, you could do a geometric
    // check between the two images that form the putative loop closure. (There
    // is no geometric check in this version of FabMap. Adding one will boost
    // accuracy a lot!). You could also use position information from a metric
    // SLAM system to rule out unlikely loop closures.
    //
    // NB: It is much much less bad to incorrectly add a new place to the map
    // than to incorrectly confirm a loop closure. If you are uncertain, simply
    // NEVER call ConfirmLastMatch() and FabMap will perform fine. Every
    // observation will then be added to the map as a new place. This will lead
    // to a very gradual drop in performance. However, confirming an incorrect
    // loop closure will break the system almost immediately. Just like in
    // metric SLAM systems, a bad data association decision will quickly cause
    // FabMap to diverge. Unlike in metric SLAM systems, never performing data
    // association works OK. See Mark Cummins' thesis for discussion.
    void ConfirmLastMatch();

    // Same as above, but the user can enforce a data association decision other
    // than that determined by FabMap.
    void ConfirmLastMatch(const int nMLplace);

    // Throw away last observation. (i.e. Neither add a new place nor update any
    // place in the map). Normally FabMap never does this, but you can request
    // this behaviour if it makes sense for your application.
    void DiscardInfoFromLastMatch();

    // Clear the map and all internal state.
    void FabMap_Restart();

    //**************************************************************************
    //                         Extended API
    // The above functions are sufficient to use FabMap. The functions below
    // provide some extra controls for specific use cases.
    //**************************************************************************

    // Add locations to the map directly. This is useful e.g. for reloading a
    // previous run of the robot or setting up a static image search index.
    void AddToObservationToMap(const Observation &raw_observation);

    // This function does a pure image "query".
    // It computes the likelihood over places in the map, without any temporal prior,
    // and doesn't update anything in the map.
    // This is useful if you want to use FabMap as a generic image search engine,
    // e.g. for Google-Goggles like object recognition against a static set of objects.
    void ProcessObservation_ComputeLikelihoodOnly(
        const Observation &raw_observation,
        LocationProbabilityContainer &ComputedLocationProbability);

    // Convenience function. Same as above but directly returns the ID of the most likely match.
    unsigned int ProcessObservation_ComputePlaceWithHighestLikelihood(const Observation &raw_observation);

private:
    //Parameter Parsing and setup
    void DoInitialSetup();
    void GenerateCachedValues();
    vector<vector<int> > ReverseTree(vnl_vector<int> Parent);

    //Main functions that orchestrate the calculation.
    void CalculatePSameMatrix();

    //Fetch observations
    void GetNextObservation(SceneRecord &NextObservation);

     //Functions to calculate PDF, decide data association and update models.
    void CalculatePDFOverLocation(const SceneRecord &this_scene, const vector<double> &location_prior);
    void DecideDataAssociation_andUpdatePlaceModels(const SceneRecord &observations);
    int DecideDataAssociation() const;

    //Some wrappers for online incremental use
    void ProcessObservation_ComputePDF(const SceneRecord &observation);

    //Methods for writing out results
    void CreateOutputFileStructures();
    void WriteResultsToDisk(string datasetName, double dfTimeTaken);
    void WriteLineOfResults(unsigned int nImageID);
    void WriteLineOfResults_Sparse(unsigned int nImageID);
    void WriteLineOfResults_SparseTopK(unsigned int nImageID);
    void WriteLineOfResults_Dense();
    void RecordConfigParamsToFile(string output_path, double dfTimeTaken);
    void RecordConfigParamsToFile(string output_path);

    //Convenience utility functions
    unsigned int GetCurrentMostLikelyPlace() const;

    //Fractional processing
    bool m_bProcessAllScenes;
    int m_nMaxNumberOfScenesToProcess;

    // Map update management.
    void AddSceneToIndex(const SceneRecord &observation);
    void AddSceneToIndex(const SceneRecord &observation, unsigned int nAssociatedPlaceID);

    // PDF update functions.
    void CalculateAppearanceLikelihoods(vnl_vector<double> &log_p_obs_given_scene,const SceneRecord &observation,const unsigned int nIndexOfFirstSample,const InvertedIndex &SamplesInvIndex,const vector<double> &SamplesNegBaseline,const SampleIterator &sbegin,const SampleIterator &send) const;
    void CalculateVotes(const SceneRecord &observation, bool bPositiveObservation, vector<double> &ReturnedVotes) const;
    void CalculateVotes_MarkovBlanket(const vector<unsigned int> &MarkovBlanket, vector<double> &ReturnedVotes) const;
    void CalculateVotes_NegativeBaseline(const SceneRecord &observation, vector<double> &ReturnedVotes) const;
    void CalculateVotes_NegativeBaseline(vector<unsigned int> &WordIDs, vector<double> &ReturnedVotes) const;
    void GetMarkovBlanket(const SceneRecord &observation,vector<unsigned int> &MarkovBlanket) const;
    void GetCompositeVoteRelativeToNegBaseline(const SceneRecord &observation, vector<double> &ReturnedVotes) const;
    void GetCompositeVoteRelativeToNegBaseline(const vector<unsigned int> &MarkovBlanket, vector<double> &ReturnedVotes) const;
    double GetNegativeBaselineForScene(const SceneRecord &observation) const;

    //Low Level Functions - these functions are the inner loop of the PDF update
    inline double SumOfLogs(const vnl_vector<double> &LogValues) const;
    inline double GetP_e_1_given_z(const unsigned int nWordID, const unsigned int state_z) const;
    inline double GetP_z_Given_Zp_and_E(const unsigned int z_id,const unsigned int state_z,const unsigned int state_e,const unsigned int state_zp) const;
    inline double GetP_z_Given_Zp(const unsigned int z_id,const unsigned int state_z,const unsigned int state_zp, const PlaceModel &word_exists_prob) const;

    // Private data.
    InvertedIndex m_WordToScenes;
    SceneRecordConatiner m_SceneRecordsInMap;
    vector<double> m_NegativeBaseline;      //Cached negative votes for each place.

    vector<unsigned int> m_SceneToPlace;    //The i-th entry is the place to which the ith observation was associated.
    unsigned int m_nNumberOfExistingPlaces; //How many places are there in the map?

    //Observations to process
    SceneRecordConatiner m_Observations;    //Old code batch-read all observations. Waste of memory - only one observation needed at any given time.
    OXSIterator* mp_OXSIterator;            //New code uses iterator to read incrementally from the file

    //Some state from the previous time step
    //to allow user to externally confirm data associations
    SceneRecord m_LastObservation;
    bool m_bLastMatchConfirmed;
    bool m_bDiscardInfoFromLastMatch;
    int m_nObservationsProcessed;

    //Useful constants
    unsigned int m_nNumInputObservations;
    const unsigned int m_nVocabSize;
    const double m_df_likelihood_smoothing_factor;
    double m_dfDataAssociationThreshold;

    //Memory management
    //Pre-allocating some arrays prevents memory fragmentation
    //and decreases memory footprint >90% in some cases.
    unsigned int m_nPreAllocateLength;

    //Prior
    LocationPriorProvider *mp_PriorProvider;
    bool m_bDisallowLocalMatches;
    unsigned int m_nDisallowRegion;

    //Sampler
    Sampler *mp_Sampler;
    unsigned int m_nMaxSamples;
    unsigned int m_nMinNumSamples;
    string m_sAltSamplePath, m_sAltSampleFile;

    //Ranking Function
    RankingFunction *mp_RankingFunction;

    //Keyframe Detector
    KeyframeDetector *mp_KeyframeDetector;
    unsigned int m_nMostLikelyPlaceLastIteration;

    //File Streams, File Locations and I/O related
    ofstream m_results_file;
    ofstream m_SceneToPlace_file;
    string m_sVocabPath, m_sVocabName;
    string m_base_output_path;
    string m_full_output_path;  //Subdirectory of base, created for this particular run.
    double m_dfBlobResponseThreshold;       //Optionally only load features from input file above a certain blob response threshold. For easy testing of the effect of varying the number of interest points per image.
    bool m_bTextOutputFormat;   //If true, we output results in a txt matrix format. Otherwise we output a dense Matlab matrix.
    bool m_bSparseOutputFormat; //Sparse or dense txt output format? No effect if we're recording to a matlab file.
    bool m_bTopKOutput;         //Record top K matches in sparse format.
    double m_dfSparseRecordingThreshold;
    double m_dfLastPDFCalculationTime;

    // Key probability values for the words of the vocabulary.
#ifdef FLOAT_PRECISION_PLACE_MODELS
    vector<float> Marginals;
#else
    // i-th entry is p(z_i = 1), the marginal probability of observing the i-th word in the visual vocabulary.
    vector<double> Marginals;
#endif
    // i-th entry is p(z_i = 1 | z_pi = 1), where pi is the parent of word i in the Chow Liu tree.
    vnl_vector<double> RelevantConditionals;
    // i-th entry is p(z_i = 0 | z_pi = 0), where pi is the parent of word i in the Chow Liu tree.
    vnl_vector<double> RelevantNotConditionals;
    //The i-th entry in the vector is the parent of i-th node in the Chow Liu tree. -1 indicates the root.
    vnl_vector<int> Parent;
    //We  also store the tree backwards, where each node stores a list of its children.
    vector<vector<int> > Children;

    //Detector models - we allow p(z | e) to be different for each word, although this isn't currently exploited.
    vector<double> p_z_1_given_e_0;
    vector<double> p_z_1_given_e_1;

    const double p_observe_given_exists, p_at_new_place;
    const double p_not_observe_given_exists; //This is (1-p_observe_given_exists).
    const double one_over_p_not_observe_given_exists; //This is 1/(1-p_observe_given_exists). Stored purely for efficiency.

    //Cached values - These values are used repeatedly by the coupled detector equations.
    vector<double> K1; //K1(i) = (Marginals(i)*p_not_observe_given_exists)/((1.0-Marginals(i))*p_observe_given_exists);

    //Calculated Quantities - The things we're working out
    LocationProbabilityContainer location_probability; //This is the pdf over location given the most recent observation.

    vnl_matrix<double> psame;    //psame(i,j) is the probability that image j came from place i. (Or loosely, the probability that images i and j came from the same place.)
};


#endif //FABMAP_H
