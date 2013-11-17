#ifndef KEYFRAMEDETECTOR_FABMAP_H
#define KEYFRAMEDETECTOR_FABMAP_H 1

#include "TypeDefinitions.h"
#include "FabMap.h"
#include <vector>

//********************************
//  KeyframeDetector Base Class
//********************************

class KeyframeDetector
{
public:
    KeyframeDetector() {};
    virtual ~KeyframeDetector() {};

    //Is KeyFrame inspects the current observation and decides if it's a new keyframe.
    //The motion prior is also inspected, because you might want to let the motion estimate dictate that a new keyframe should be created,
    //regardless of the appearance similarity. Currently this motion information is only used by FabMapKeyframeDetector.
    virtual bool IsKeyframe(const SceneRecord &observation, const vector<double> &prior, const unsigned int nSceneId) = 0;
    virtual void WriteKeyframesToDisk(const std::string &sDatasetName,const std::string &sPath) const;
    virtual std::string DescribeKeyframeDetector() const = 0;
    
protected:
    std::vector<unsigned int> m_ChosenKeyframes;
};

//*************************************
//  Null Keyframe Detector
//*************************************
//A dummy class to disable keyframe detection
//Always declare a keyframe
//Don't write out a list of keyframes
class NullKeyframeDetector : public KeyframeDetector
{
public:
    NullKeyframeDetector() {};
    virtual ~NullKeyframeDetector() {};
    bool IsKeyframe(const SceneRecord &observation, const vector<double> &prior, const unsigned int nSceneId);
    void WriteKeyframesToDisk(const std::string &sDatasetName,const std::string &sPath) const;
    std::string DescribeKeyframeDetector() const;
};

//*************************************
//  Fixed Increment Keyframe Detector
//*************************************
//Fixed increment keyframes
//Declare a keyframe every N frames

class FixedIncrementKeyframeDetector : public KeyframeDetector
{
public:
    FixedIncrementKeyframeDetector(int nIncrement, unsigned int nPreAllocateLength = 50000);
    virtual ~FixedIncrementKeyframeDetector() {};

    bool IsKeyframe(const SceneRecord &observation, const vector<double> &prior, const unsigned int nSceneId);
    std::string DescribeKeyframeDetector() const;

protected:
    int m_nFrameCount;
    const unsigned int m_nIncrement;    //Declare a new keyframe every nIncrement frames.
};

//*************************************
//  Words-In-Common Keyframe Detector
//*************************************
//Keyframes based on counting number of words in common with last observation
//If the percentage of words in common with the last observation drops below 
//a threshold, declare a new keyframe.

class WordsInCommonKeyframeDetector : public KeyframeDetector
{
public:
    WordsInCommonKeyframeDetector(double dfPercentageWordsInCommonThreshold, unsigned int nPreAllocateLength = 50000);
    virtual ~WordsInCommonKeyframeDetector() {};

    bool IsKeyframe(const SceneRecord &observation, const vector<double> &prior, const unsigned int nSceneId);
    std::string DescribeKeyframeDetector() const;

protected:
    bool bFirstObservationSet;
    const double m_dfPercentageWordsInCommonThreshold;
    std::vector<unsigned int> m_WordsIDsInLastScene;
};

//*************************************
//     FabMap Keyframe Detector
//*************************************
//Declare a new keyframe when FabMap says the most
//likely place is new, or somewhere other than 
//the most likely place due to the last keyframe
//
//This is expensive, because each keyframe test requires
//us to compute the pdf over places, so it is just as expensive 
//a normal observation.

class FabMapKeyframeDetector : public KeyframeDetector
{
public:
    FabMapKeyframeDetector(FabMapCalculator &i_world);
    virtual ~FabMapKeyframeDetector() {};

    bool IsKeyframe(const SceneRecord &observation, const vector<double> &prior, const unsigned int nSceneId);
    std::string DescribeKeyframeDetector() const;

private:
    FabMapCalculator &m_world;
    unsigned int m_nMostLikelyPlace_LastKeyframe;
};

#endif //KEYFRAMEDETECTOR_FABMAP_H
