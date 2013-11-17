#include "KeyframeDetector.h"
#include "MAT_parser.h"
#include <sstream>
using namespace std;

//********************************
//  Keyframe Detector Base Class
//********************************
void KeyframeDetector::WriteKeyframesToDisk(const std::string &sDatasetName,const std::string &sPath) const
{
    WriteToFile(m_ChosenKeyframes,"keyframes",sDatasetName,sPath);
}

//********************************
//     Null Keyframe Detector 
//********************************
bool NullKeyframeDetector::IsKeyframe(const SceneRecord &observation, const vector<double> &prior, const unsigned int nSceneId)
{
    return true;
}

void NullKeyframeDetector::WriteKeyframesToDisk(const std::string &sDatasetName,const std::string &sPath) const
{
    //Keyframe detection is effectively disabled (every frame is a keyframe).
    //Don't write out a keyframe index
}

string NullKeyframeDetector::DescribeKeyframeDetector() const
{
    return "NONE";
}

//*************************************
//  Fixed Increment Keyframe Detector
//*************************************

FixedIncrementKeyframeDetector::FixedIncrementKeyframeDetector(int nIncrement, unsigned int nPreAllocateLength)
:m_nFrameCount(0),m_nIncrement(nIncrement)
{
    m_ChosenKeyframes.reserve(nPreAllocateLength);
}


bool FixedIncrementKeyframeDetector::IsKeyframe(const SceneRecord &observation, const vector<double> &prior, const unsigned int nSceneId)
{
    if(--m_nFrameCount <= 0)
    {
        m_nFrameCount = m_nIncrement;
        m_ChosenKeyframes.push_back(nSceneId);
        return true;
    }
    else
    {
        return false;
    }
}

string FixedIncrementKeyframeDetector::DescribeKeyframeDetector() const
{
    stringstream sstr;
    sstr << "FIXED_INCREMENT, with increment " << m_nIncrement;
    return sstr.str();
}
//*************************************
//  Words-In-Common Keyframe Detector
//*************************************

WordsInCommonKeyframeDetector::WordsInCommonKeyframeDetector(double dfPercentageWordsInCommonThreshold, unsigned int nPreAllocateLength)
:bFirstObservationSet(false),m_dfPercentageWordsInCommonThreshold(dfPercentageWordsInCommonThreshold)
{
    m_ChosenKeyframes.reserve(nPreAllocateLength);
}

string WordsInCommonKeyframeDetector::DescribeKeyframeDetector() const
{
    stringstream sstr;
    sstr << "WORDS_IN_COMMON, with threshold " << m_dfPercentageWordsInCommonThreshold;
    return sstr.str();
}

bool WordsInCommonKeyframeDetector::IsKeyframe(const SceneRecord &observation, const vector<double> &prior, const unsigned int nSceneId)
{
    if(!bFirstObservationSet)
    {
        //This is the first observation we've ever seen. Declare it a keyframe and store its words.
        for(unsigned int i=0;i<observation.WordData.size();i++)
            m_WordsIDsInLastScene.push_back(observation.WordData[i].nID);

        bFirstObservationSet = true;
        m_ChosenKeyframes.push_back(nSceneId);
        return true;
    }
    else
    {
        //Declare it a keyframe if the percentage of words in common with the last keyframe
        //is below the threshold

        //Intersect words in this scene with words in last keyframe
        vector<unsigned int> WordsIDsInScene;
        const unsigned nNumWords = observation.WordData.size();
        WordsIDsInScene.reserve(nNumWords);
        for(unsigned int i=0;i<nNumWords;i++)
            WordsIDsInScene.push_back(observation.WordData[i].nID);
      
        vector<unsigned int> WordsInCommon;
        std::set_intersection(WordsIDsInScene.begin(),WordsIDsInScene.end(),
                              m_WordsIDsInLastScene.begin(),m_WordsIDsInLastScene.end(),
                              std::back_inserter(WordsInCommon));

        double dfNumWordsThisScene = (double) WordsIDsInScene.size();
        double dfNumWordsLastScene = (double) m_WordsIDsInLastScene.size();
        double dfNumWordsInCommon  = (double) WordsInCommon.size();
        double dfPercentInCommon = dfNumWordsInCommon/(dfNumWordsThisScene+dfNumWordsLastScene);

        //If percent in common has dropped below threshold, declare a new keyframe
        if(dfPercentInCommon < m_dfPercentageWordsInCommonThreshold)
        {
            //Store words occuring as last scene
            m_WordsIDsInLastScene = WordsIDsInScene;
            m_ChosenKeyframes.push_back(nSceneId);
            return true;
        }
        else
        {
            return false;
        }
    }
}

//*************************************
//     FabMap Keyframe Detector
//*************************************
FabMapKeyframeDetector::FabMapKeyframeDetector(FabMapCalculator &i_world)
: m_world(i_world),m_nMostLikelyPlace_LastKeyframe(0)
{
    m_ChosenKeyframes.reserve(i_world.m_nPreAllocateLength);
    //FabMap always accepts the first observation as a keyframe
    m_ChosenKeyframes.push_back(1);
}

string FabMapKeyframeDetector::DescribeKeyframeDetector() const
{
    return "FABMAP_LIKELIHOOD";
}

bool FabMapKeyframeDetector::IsKeyframe(const SceneRecord &observation, const vector<double> &prior, const unsigned int nSceneId)
{
    //Calculate the PDF over observations due to the observation
    m_world.CalculatePDFOverLocation(observation,prior);

    //Determine most likely place
    LocationProbabilityContainer::iterator max_pos = max_element(m_world.location_probability.begin(),m_world.location_probability.end());
    unsigned int ml_place = distance(m_world.location_probability.begin(),max_pos); 

    //If the most likely place is a new place
    //declare a new keyframe
    //Also, if the most likely place is not a new place, but not the most likely place due to the last keyframe, and not the most recent place added to the map
    //declare a new keyframe
    if(ml_place == m_world.m_nNumberOfExistingPlaces || ((ml_place != m_world.m_nNumberOfExistingPlaces) && (ml_place != m_nMostLikelyPlace_LastKeyframe) && (ml_place != m_world.m_nNumberOfExistingPlaces-1)))
    {
        m_nMostLikelyPlace_LastKeyframe = ml_place;
        m_ChosenKeyframes.push_back(nSceneId);
        return true;
    }
    else
    {
        return false;
    }
}
