#ifndef TYPE_DEFINITIONS_288818995887AHZATR_H
#define TYPE_DEFINITIONS_288818995887AHZATR_H 1

#include <vector>
#include <deque>
#include <algorithm>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>


//******************************
//      FabMapV1 Structures    *
//******************************
typedef std::vector<unsigned int> Observation;          //Size of vocab, each entry 0 or 1

#ifdef FLOAT_PRECISION_PLACE_MODELS
    typedef std::vector<float> PlaceModel;
#else
    typedef std::vector<double> PlaceModel;
#endif

//******************************
//      FabMapV2 Structures    *
//******************************

//Observation
struct WordWithOccurences
{
    unsigned int nID;   //Which word?
    unsigned int nN;    //How many instances?
};

typedef std::vector<WordWithOccurences> SparseObservation;    //Only record positive observations. All wordIDs not present were implicitly seen 0 times.

//Inverted index
struct InvertedIndexEntry
{
    unsigned int nSceneID;
    unsigned int nN;
};

//Scenes
struct InterestPoint
{
    unsigned int camera_id;    //For multi-part images, eg Ladybug, which camera was the interest point detected in
    float blob_response; //Strength of interest point reported by SURF
    unsigned int x;
    unsigned int y;
    float s;     //Scale
};

class WordWithInterestPoints
{
public:
    unsigned int nID;                       //Which word?
    unsigned int nN;                        //How many instances?
    std::vector<InterestPoint> InterestPoints;   //Their physical locations

    WordWithInterestPoints()
    {}

    //These constructors are for dealing with old OXS data where we have no InterestPoint location information.
    WordWithInterestPoints(const WordWithOccurences &wwo)
        : nID(wwo.nID), nN(wwo.nN)
    {}
    WordWithInterestPoints(unsigned int i_nID,unsigned int i_nN)
        : nID(i_nID), nN(i_nN)
    {}
    WordWithInterestPoints(unsigned int i_nID)
        : nID(i_nID)
    {}

    bool operator< (const WordWithInterestPoints &other) const { return nID < other.nID; }
};

class SceneRecord
{
public:
    unsigned int nNumWords;
    std::vector<WordWithInterestPoints> WordData;
    unsigned int nTotalNumInterestPointsInScene;

    //Default constructor
    SceneRecord() {}

    //This constructior is for dealing with old MOOS messages, which don't even contain data on how often a word occurred.
    SceneRecord(const std::vector<unsigned int> &word_ids)
    {
        nNumWords = word_ids.size();
        nTotalNumInterestPointsInScene = 0;
        WordData.reserve(nNumWords);
        for(unsigned int i=0;i<nNumWords;++i)
        {
            WordData.push_back(WordWithInterestPoints(word_ids[i],1)); //No data, so assume one occurrence of the word
            nTotalNumInterestPointsInScene += 1;
        }
        //Ensure WordData is sorted
        std::sort(WordData.begin(),WordData.end());
    }

};

//ContainerTypes
typedef std::vector<double> LocationProbabilityContainer;
typedef std::vector< std::vector<InvertedIndexEntry> > InvertedIndex;
typedef std::deque<SceneRecord> SceneRecordConatiner;

//******************************
//           Common            *
//******************************

typedef std::vector<boost::numeric::ublas::compressed_vector<unsigned int> > SparseMatrix;

//Struct to store the details of the raw data used to generate the observations
//We send this info out in loop closure messages, to support displaying the results, etc.
struct RawDataDetails
{
    double dfTime;
    std::string sID;    //Cache ID
    int nTransactionID;
    std::string sFilename;
    int nImageWidth;
    int nImageHeight;

};

#endif //TYPE_DEFINITIONS_288818995887AHZATR_H
