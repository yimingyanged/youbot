#ifndef OXS_parser_helper_h
#define OXS_parser_helper_h

#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <TypeDefinitions.h>
#include "MOOSGenLib/MOOSGenLibGlobalHelper.h"
using namespace std;

//***************************************
//         Tokenization
//Fast functor for string tokenization
//***************************************
class StringTokenizer
{
public:
    StringTokenizer(string sStr, char cSep=' ') : m_sStr(sStr), m_cSep(cSep), m_nPos(0), m_nMax(sStr.size()) {}

    bool HasNext() {return (m_nPos < m_nMax-1) || (m_nPos == (m_nMax-1) &&  m_sStr[m_nPos] != m_cSep);}
protected:
    string m_sStr;
    const char m_cSep;
    string m_sTmp;
    int m_nPos;
    int m_nMax;
};

class StringTokenizer_ToInt : public StringTokenizer
{
public:
    StringTokenizer_ToInt(string sStr,char cSep=' ') 
        : StringTokenizer(sStr,cSep)
    {}   

    int operator () ()
    {
        m_sTmp.clear();
        while(m_nPos<m_nMax && m_sStr[m_nPos]!=m_cSep)
            m_sTmp.push_back(m_sStr[m_nPos++]);
        m_nPos++;   //Finally, advance past the seperator
        return atoi(m_sTmp.c_str());
    }
};

class StringTokenizer_ToFloat : public StringTokenizer
{
public:
    StringTokenizer_ToFloat(string sStr,char cSep=' ') 
        : StringTokenizer(sStr,cSep)
    {}   

    float operator() ()
    {
        m_sTmp.clear();
        while(m_nPos<m_nMax && m_sStr[m_nPos]!=m_cSep)
            m_sTmp.push_back(m_sStr[m_nPos++]);
        m_nPos++;   //Finally, advance past the seperator
        return ((float) atof(m_sTmp.c_str()));
    }
};


//***************************************
//         Filtering
// Helper functions to exclude IPs
// in certain regions of the image
//***************************************

//Used for specifying regions of the image to exclude from processing.
struct ImageRegion
{
    float x_low;
    float x_high;
    float y_low;
    float y_high;
};

vector<vector<ImageRegion> > LoadExcludedRegions(string sExcludedRegionsFilePath);

//Functor for checking that an interest point is not in an excluded region
class InterestPointIsInExcludedRegionCheck
{
public:
    InterestPointIsInExcludedRegionCheck (){}

    InterestPointIsInExcludedRegionCheck (string sExcludedRegionsFilePath)
    {
        if(!sExcludedRegionsFilePath.empty())
            m_ExcludedRegions = LoadExcludedRegions(sExcludedRegionsFilePath);
    }

    InterestPointIsInExcludedRegionCheck (vector<vector<ImageRegion> >  ExcludedRegions)
    : m_ExcludedRegions(ExcludedRegions) {}

    bool operator () (const InterestPoint ipoint) const
    {
        bool bPointIsInExcludedRegion = false;
        if(ipoint.camera_id < m_ExcludedRegions.size())
        {
            for(unsigned int r=0;r<m_ExcludedRegions[ipoint.camera_id].size();++r)
            {
                ImageRegion region = m_ExcludedRegions[ipoint.camera_id][r];
                bPointIsInExcludedRegion |=  (ipoint.x > region.x_low && ipoint.x < region.x_high) && (ipoint.y > region.y_low && ipoint.y < region.y_high); //Check if it's in the r-th region.
            }
        }
        return bPointIsInExcludedRegion;
    }

    bool empty() const { return m_ExcludedRegions.empty(); }
private:
    vector<vector<ImageRegion> > m_ExcludedRegions;
};

//***************************************
//         Read A Record
//  Key shared parsing code.
//  Read a record from a file
//***************************************

//Functor to read a record from an OXS file.
//More efficient to make this a functor - can avoid large temporary string variables being reassigned at each call.
class ReadRecord
{
public: 
    ReadRecord(ifstream& File, const bool i_bLoadGeometry, const bool i_bNewOXSFormat, const bool i_bFiltersApplied, const bool i_bFilterBlobResponse, const double i_dfBlobThreshold,const InterestPointIsInExcludedRegionCheck &i_IPExclusionCheck, const unsigned int i_nLadybugImageWidth)
        : ScenesFile(File), bLoadGeometry(i_bLoadGeometry), bNewOXSFormat(i_bNewOXSFormat), bFiltersApplied(i_bFiltersApplied), bFilterBlobResponse(i_bFilterBlobResponse),
          dfBlobThreshold(i_dfBlobThreshold), IPExclusionCheck(i_IPExclusionCheck), nLadybugImageWidth(i_nLadybugImageWidth)
    {}

    bool operator () (SceneRecord &CurrentScene)
    {
        bool bRecordFound = false;

        //Clear everything
        CurrentScene.nNumWords = 0;
        CurrentScene.nTotalNumInterestPointsInScene = 0;
        CurrentScene.WordData.clear();

        while(!ScenesFile.eof() && !bRecordFound)
        {
            //grab a line
            getline(ScenesFile,sLine,'\n');
            //Look for the keyword we want
            size_t nPos = sLine.find("SCENE:");
            if(nPos!=std::string::npos)
            {
                bRecordFound = true;
                //Skip 2 lines, then grab the data
                getline(ScenesFile,sLine,'\n');getline(ScenesFile,sLine,'\n');
                //Read what words occured in this scene
                getline(ScenesFile,sWhichWords,'\n');
                //And how often
                getline(ScenesFile,sHowOften,'\n');

                //Defend against infuriating Linux/Windows line ending differences
                MOOSRemoveChars(sWhichWords,"\r");
                MOOSRemoveChars(sHowOften,"\r");

                //If the OXS file has geometry info, and we want it, go read that too
                if((bLoadGeometry || bFiltersApplied) && bNewOXSFormat)
                {
                    getline(ScenesFile,sCameraID,'\n');
                    getline(ScenesFile,sBlobResponse,'\n');
                    getline(ScenesFile,sX,'\n');
                    getline(ScenesFile,sY,'\n');
                    getline(ScenesFile,sScale,'\n');
                    MOOSRemoveChars(sCameraID,"\r");
                    MOOSRemoveChars(sBlobResponse,"\r");
                    MOOSRemoveChars(sX,"\r");
                    MOOSRemoveChars(sY,"\r");
                    MOOSRemoveChars(sScale,"\r");
                }

                //Set up tokenziers
                StringTokenizer_ToInt WhichWords_Stream(sWhichWords);
                StringTokenizer_ToInt HowOften_Stream(sHowOften);
                StringTokenizer_ToInt CameraID_Stream(sCameraID);
                StringTokenizer_ToFloat BlobResponse_Stream(sBlobResponse);
                StringTokenizer_ToInt X_Stream(sX);
                StringTokenizer_ToInt Y_Stream(sY);
                StringTokenizer_ToFloat Scale_Stream(sScale);

                unsigned int nTotalInterestPointsInScene = 0;
                while(WhichWords_Stream.HasNext())
                {
                    //And then parse the the result and put into some convenient structure
                    WordWithInterestPoints wwip;
                    wwip.nID = WhichWords_Stream();
                    wwip.nN = HowOften_Stream();
                    //If we're filtering on blob response, need to record that
                    unsigned int nNumInterestPointsAboveThreshold = 0;
                    //If geometry present, and we want it, go read that as well
                    if(bLoadGeometry && bNewOXSFormat)
                    {
                        for(unsigned int g =0;g<wwip.nN;++g)
                        {
                            InterestPoint ip;
                            ip.camera_id = CameraID_Stream();
                            ip.blob_response = BlobResponse_Stream();
                            ip.x = X_Stream();
                            ip.y = Y_Stream();
                            ip.s = Scale_Stream();
                            if(!bFiltersApplied || ( (!bFilterBlobResponse || (bFilterBlobResponse && ip.blob_response>=dfBlobThreshold)) &&  !IPExclusionCheck(ip) ) )
                            {
                                ip.x += ip.camera_id*nLadybugImageWidth; //For Ladybug, x-position is relative to the side of each CCD. Convert here into absolute composite image coords. Single-CCD camera coords don't get changed.
                                wwip.InterestPoints.push_back(ip);
                            }
                        }
                        nNumInterestPointsAboveThreshold = wwip.InterestPoints.size();
                        //Trim excess capacity from the vector using the swap trick. Worthwhile, because these vectors will never change size.
                        vector<InterestPoint>(wwip.InterestPoints).swap(wwip.InterestPoints);                        
                    }
                    //If we're not loading geometry, but do want to filter, need to go read the blob responses and/or geometry
                    if(!bLoadGeometry && bFiltersApplied && bNewOXSFormat)
                    {                   
                        if(bFilterBlobResponse && IPExclusionCheck.empty())     //Only filtering on blob response
                        {
                            for(unsigned int g =0;g<wwip.nN;++g)
                            {
                                double dfBlob = BlobResponse_Stream();
                                if(dfBlob >= dfBlobThreshold)
                                    nNumInterestPointsAboveThreshold++;
                            }
                        }
                        else if(!bFilterBlobResponse && !IPExclusionCheck.empty()) //Only filtering on geometry
                        {
                            for(unsigned int g =0;g<wwip.nN;++g)
                            {
                                InterestPoint ip;
                                ip.camera_id = CameraID_Stream();
                                ip.x = X_Stream();
                                ip.y = Y_Stream();
                                if(!IPExclusionCheck(ip))
                                    nNumInterestPointsAboveThreshold++;
                            }
                        }
                        else                                                        //Filtering on both
                        {
                            for(unsigned int g =0;g<wwip.nN;++g)
                            {
                                InterestPoint ip;
                                ip.camera_id = CameraID_Stream();
                                ip.x = X_Stream();
                                ip.y = Y_Stream();
                                double dfBlob = BlobResponse_Stream();
                                if((dfBlob >= dfBlobThreshold) && !IPExclusionCheck(ip))
                                    nNumInterestPointsAboveThreshold++;
                            }
                        }
                    }

                    //If we're filtering out low-blob-response interest points, adjust the interest point count accordingly
                    if(bNewOXSFormat && bFiltersApplied)
                        wwip.nN = nNumInterestPointsAboveThreshold;

                    //Store the parsing results, unless we're filtering and the blob reponse is below the threshold
                    if(!bNewOXSFormat || ( bNewOXSFormat && !bFiltersApplied )|| 
                       (bNewOXSFormat && bFiltersApplied && nNumInterestPointsAboveThreshold!=0))
                    {
                        CurrentScene.WordData.push_back(wwip);
                        nTotalInterestPointsInScene += wwip.nN;
                    }
                }
                CurrentScene.nTotalNumInterestPointsInScene = nTotalInterestPointsInScene;
                CurrentScene.nNumWords = CurrentScene.WordData.size();
                std::sort(CurrentScene.WordData.begin(),CurrentScene.WordData.end());   //Paranoia. Ensure words are sorted.
                //Trim excess capacity from the vector using the swap trick.
                vector<WordWithInterestPoints>(CurrentScene.WordData).swap(CurrentScene.WordData);                        

            }
        }
        return bRecordFound;
    }

private:
    ifstream& ScenesFile;
    bool bLoadGeometry, bNewOXSFormat, bFiltersApplied, bFilterBlobResponse;
    double dfBlobThreshold;
    
    InterestPointIsInExcludedRegionCheck IPExclusionCheck;
    string sLine, sWhichWords, sHowOften, sCameraID, sBlobResponse, sX, sY, sScale; //Scale is 1/r^2
    unsigned int nLadybugImageWidth;
};


#endif //OXS_parser_helper_h
