#ifndef __WordMaker_h
#define __WordMaker_h

#include <vector>
#include <set>
#include <iostream>
#include <iomanip>

// Data types
#include "WordMakerDataTypes.h"

//KD-Tree
#include "nn.h"

//SURF
//#ifdef WIN32
//#include "surfWINDLL.h"
//#endif
//#include "imload.h"


// VNL
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

// MOOS
#include <MOOSLIB/MOOSLib.h>
#include <MOOSGenLib/MOOSGenLibGlobalHelper.h>





//Support for creating directories
#ifdef WIN32
  #include <direct.h>
  #define mkdir(a) _mkdir(a)
#endif

//File IO helper functions
#include "IOHelperFunctions.h"

//Can't get libjpeg to work - relying on ImageMagick
//#define cimg_use_jpeg    
//#include "CImg.h"        //This header file seems to cause problems if not included last.
//using namespace cimg_library;


// generic SURFlike feature extraction interface
#include "InterestPoint.h"
#include "SurfTools.h"


using namespace std;
using namespace SurfInterface;

class CWordMaker : public CMOOSApp
{
public:
    // Construction / Destruction
    CWordMaker();
    virtual ~CWordMaker();
    
    //*******************************************************************************
    //                            Easy batch processing.
    // Run WordMaker from the commandline to process all the images in a directory.
    //*******************************************************************************
    void BatchProcess(string sAppName, string sMissionFile);
    void GenerateSURFsOnly(string sAppName, string sMissionFile);
    void QuantizeOnly(string sAppName, string sMissionFile);

    //*******************************************************************************
    //                               External API.
    // Use a WordMaker object from within other code. C++ and Python bindings.
    //*******************************************************************************

    // Setup - specify the visual vocabulary settings to use.
    bool LoadVocabularyAndSetup(const string &sVocabPath,
                                const string &sVocabName,
                                const unsigned int nDescriptorDimensionality);

    // Setup - specify the threshold used by the SURF detector. Default is 25.
    void SetSURFThreshold(const double SURF_thres);

    // Setup - specify whether the SURF detector should use uSURF (no rotation invariance)
    // and/or SURF128 (2x longer descriptor).
    // Default is use uSURF 128 - i.e. longer descriptor without rotation invariance.
    void SetSURFOptions(const bool SURF_upright,
                        const bool bSURF128);

    // Given the path to an image file, compute a bag-of-words descriptor.
    bool GetBagOfWordsForImage(const string &sImagePath_andFilename,
                               vector<int> &ReturnedWords);

    // Given the path to an image file, compute a bag-of-words descriptor.
    // Also return the details of the interest point locations.
    bool GetBagOfWordsForImage(const string &sImagePath_andFilename,
                               vector<int> &ReturnedWords,
                               BagOfRegions &RegionBag);

    // Given a vector of feature vectors directly, compute the bag of words.
    bool QuantizeFeatures(vector<vector<float> > &FeatureVectors,
                          vector<int> &ReturnedWords);

private:
    void DoInitialSetup();
    bool InitializeKDTree();
    //File IO
    bool ReadVocabularyFile();
    bool ReadOXV(); 
    inline void ChompVocabFile(ifstream & ifs, vnl_vector<float>& D) const;
    void saveIpoints(const string &sFileName, const vector< InterestPoint >& ipts_view) const;
    void ReadFeatureFile(const string &sFile, vector<PatchDescriptor> &ReturnedDescriptors) const;
    void ReadTxtFeatureFile(const string &sFile, vector<PatchDescriptor> &ReturnedDescriptors) const;
    void SetDescriptorLengthFromFeatureFile(const string &sFile);
    void WriteOXSLine(ofstream & oxs, const vector<int>& ReturnedWords,const BagOfRegions &RegionBag,const std::string &sImageName) const;
    unsigned int FindNumImagesPerScene(const list<string> &sContents) const;
    //File IO for INRIA Holidays
    bool bFormatINRIA;  //Descriptors in INRIA binary format, or Oxford text format?
    bool ReadFVecs();
    bool ReadSIFTGeo(const string &sFile, vector<PatchDescriptor> &ReturnedDescriptors) const;

    //Functions to digest images
    void ProduceBagOfWords(string &filePath, const string &sImageName, vector<int> &ReturnedWords) const;
    void ProduceBagOfWords(string &filePath, const string &sImageName, vector<int> &ReturnedWords,BagOfRegions &RegionBag, const unsigned int nCurrentPartID) const;
    void QuantizeDescriptors(const vector< InterestPoint >& ipts_view, const string &sImageName, vector<int> &ReturnedWords,BagOfRegions &RegionBag, const unsigned int nCurrentPartID) const;
    void QuantizeDescriptors(const vector<PatchDescriptor>& Descriptors, const string &sImageName, vector<int> &ReturnedWords,BagOfRegions &RegionBag, const unsigned int nCurrentPartID) const;
    void DiscardIptsInExcludedRegions(vector<InterestPoint>& ipts_view,const unsigned int nCurrentPartID) const;

    //Convert vector format for returns to external callers
    void ConvertReturnedWordsFromBinaryToIndexFormat(const vector<int> &ReturnedWords_Binary,vector<int> &ReturnedWords_Index);

    // Members
    std::vector< vnl_vector<float> > m_Vocab;
    double m_dfClusterThreshold;
    Index m_LNNIndex; //KD Tree
    unsigned int m_nNumkDTrees;
    unsigned int m_kDTreeNodesToCheck;
    vnl_matrix<float> m_LNNData;
    vector<vector<ImageRegion> > ExcludedRegions;

    //SURF parameters
    double m_SURF_thres;            // blob response threshold 
    bool m_SURF_doubleImageSize;    // Initial doubling?    
    int m_SURF_initLobe;            // 3 times lobe size equals the mask size 
    int m_SURF_samplingStep;        // subsample the blob response map 
    int m_SURF_octaves;             // number of octaves to be analysed
    bool m_SURF_upright;            // true to turn off rotation invariance
    bool m_SURF_extended;           // true for SURF-128 instead of SURF-64
    int m_SURF_indexSize;           // square size of the descriptor window (default 4x4)
    unsigned int m_descriptor_length;        // 64 or 128

    //File paths
    string m_sVocabPath, m_sVocabName;
    string m_sBaseDataPath;
    string m_sImagePath;
    string m_sDescriptorPath;
    bool m_bWriteDescriptors;
    string m_sWordOutputPath;
    bool m_bWriteWords;

};

//For filename parsing
class NotImage
{
public:
    bool operator () (const string& sFilename);
};

class NotDescriptor
{
public:
    bool operator () (const string& sFilename);
};

#endif //WordMaker_h
