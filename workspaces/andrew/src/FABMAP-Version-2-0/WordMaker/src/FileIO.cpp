#include "WordMaker.h"
#include <iostream>
#include <sstream>
#include <iterator>
#include <cstdio>
#include "InterestPoint.h"

bool CWordMaker::ReadVocabularyFile()
{
    if(bFormatINRIA)
        return ReadFVecs();         //INRIA binary format
    else
        return ReadOXV();           //Oxford text format
}

//For reading in .OXV file that contains a vocabulary
bool CWordMaker::ReadOXV()
{
    ifstream ifs((m_sVocabPath + m_sVocabName + ".oxv").c_str());
    if(!ifs)
        return MOOSFail("Failed to open %s for reading\n",(m_sVocabPath + m_sVocabName + ".oxv").c_str());

    //read words
    string sLine;
    getline(ifs,sLine);        
    MOOSChomp(sLine,":");
    //How many clusters are there?
    int nClusters = atoi(sLine.c_str());
    m_Vocab.reserve(nClusters);
    //What's the threshold?
    std::getline(ifs,sLine);        
    MOOSChomp(sLine,":");
    m_dfClusterThreshold = atof(sLine.c_str());
    if(m_dfClusterThreshold < 0)
        m_dfClusterThreshold = 2.0;     //KMeans clusters have no "threshold". In the .oxv file it's specified as -1. Need to set some dummy value here.

    MOOSTrace("\nLOADING VOCABULARY:  (%d Words)....\n",nClusters);
    for( int i = 0;i<nClusters;i++)
    {
        vnl_vector<float> WordCentre;
        ChompVocabFile(ifs,WordCentre);
        m_Vocab.push_back(WordCentre);
    }

    ifs.close();
    MOOSTrace("Done\n");
    return true;
}

//Read a vocabulary file in the raw format used by INRIA Holiday Dataset
bool CWordMaker::ReadFVecs()
{
    FILE *dataFile;
    dataFile = fopen((m_sVocabPath + m_sVocabName + ".fvecs").c_str(),"rb");

    if(dataFile == NULL)
    {
        cout << "Error opening FVecs vocab file" << endl;
        return false;
    }

    // Get the descriptor length
    int d;
    if(fread(&(d),sizeof(int),1,dataFile) != 1)
        cout << "Unable to read descriptor length from FVecs file";
    else
        cout << "FVecs descriptor length is " << d;
    m_descriptor_length = (unsigned int) d;

    //Count the number of descriptors in the file
    fseek(dataFile,0,SEEK_END);
    int nClusters =  ftell(dataFile)/(1*4 + d*4);    //Each record is one int specifying length, then d floats
    fseek(dataFile,0,SEEK_SET);

    //Misc setup
    m_Vocab.reserve(nClusters);
    m_dfClusterThreshold = 2.0;     //FVecs clusters have no "threshold". Set some dummy value.


    //Read the elements
    cout << endl << "LOADING VOCABULARY:  (" << nClusters << "  Words)...." << endl;
    for(int i=0;i<nClusters;i++)
    {
        fread(&(d),sizeof(int),1,dataFile);
        vnl_vector<float> WordCentre(d);
        fread(WordCentre.data_block(),sizeof(float),d,dataFile);
        m_Vocab.push_back(WordCentre);
    }

    fclose(dataFile);
    return true;
}

void CWordMaker::ReadFeatureFile(const string &sFile, vector<PatchDescriptor> &ReturnedDescriptors) const
{
    if(!bFormatINRIA)
        ReadTxtFeatureFile(sFile,ReturnedDescriptors);
    else
        ReadSIFTGeo(sFile,ReturnedDescriptors);
}

//Read a descriptor file in the raw format used by INRIA Holiday Dataset
bool CWordMaker::ReadSIFTGeo(const string &sFile, vector<PatchDescriptor> &ReturnedDescriptors) const
{
    FILE *dataFile;
    dataFile = fopen(sFile.c_str(),"rb");

    if(dataFile == NULL)
    {
        cout << "Error opening siftgeo file" << endl << sFile << endl;
        return false;
    }

    //Count the number of descriptors in the file
    fseek(dataFile,0,SEEK_END);
    int nDescriptors =  ftell(dataFile)/(9*4 + 1*4 + 128);    //Each record is nine floats for x,y,scale,angle,afine matrix 11,12,21,22, blob response, int for descriptor dimension, and byte*descripto dim for the actual descriptor
    fseek(dataFile,0,SEEK_SET);

    //Read the descriptors
    ReturnedDescriptors.reserve(nDescriptors);
    PatchDescriptor Feature(6,128); //Throw away some info from the file. Not using affine matrix info.
    float interest_point[9];
    int dimensionality;
    char descriptor[128];
    for(int i=0;i<nDescriptors;i++)
    {
        fread(&(interest_point[0]),sizeof(float),9,dataFile);   //Read interest point geometric info
        fread(&(dimensionality),sizeof(int),1,dataFile);        //Read descriptor dimensionality
        fread(&(descriptor[0]),sizeof(char),128,dataFile);      //Read descriptor itself

        //INRIA lists descriptor details in a different order. Permute
        Feature.RegionDescriptor[0] = interest_point[8];    
        Feature.RegionDescriptor[1] = interest_point[0];
        Feature.RegionDescriptor[2] = interest_point[1];
        Feature.RegionDescriptor[3] = interest_point[2];
        Feature.RegionDescriptor[4] = interest_point[3];
        Feature.RegionDescriptor[5] = 0.0;

        //Cast descriptor bytes to floats
        for(unsigned int i=0;i<128;i++)
            Feature.FeatureDescriptor[i] = (float) descriptor[i];

        ReturnedDescriptors.push_back(Feature);
    }

    fclose(dataFile);
    return true;
}

inline void CWordMaker::ChompVocabFile(ifstream & ifs, vnl_vector<float>& D) const
{
    string sLine;
    
    //discard header, WORD:N
    std::getline( ifs, sLine );
   
    //Read in descriptor
    std::getline( ifs, sLine );
    std::istringstream sC(sLine);
    std::vector<float> T;
    T.reserve(m_descriptor_length);
    std::copy(std::istream_iterator<float>(sC),std::istream_iterator<float>(),std::back_inserter(T));
    D.set_size(T.size());
    std::copy(T.begin(),T.end(),D.data_block());
}

//For reading in .surf or .sift file that contains features
//in Mikolajczyk's format
void CWordMaker::ReadTxtFeatureFile(const string &sFile, vector<PatchDescriptor> &ReturnedDescriptors) const
{
    ifstream ifs(sFile.c_str());
    if(!ifs)
        MOOSTrace("Failed to open %s for reading\n", sFile.c_str());

    //read dimensions of the descriptors
    string sLine;
    getline(ifs,sLine);        
    int nDim = atoi(sLine.c_str());
    if(nDim != m_descriptor_length)
    {
        MOOSTrace("Error: Descriptor of unexpected length. %d instead of %d\n", nDim, m_descriptor_length);
    }
    //How many descriptors are there?
    getline(ifs,sLine);        
    int nNumDescriptors = atoi(sLine.c_str());

    //Make the values we'll return
    //vector<vnl_vector<float> > Descriptors;
    ReturnedDescriptors.reserve(nNumDescriptors);

    if(nNumDescriptors>0)
    {
        //Check if the descriptor is prepended with image co-ordinates
        //In Mikolajczyk's format, the vector in the text file can have image region specification before the descriptor
        //Typically this is 5 numbers.
        //Check for this by comparing announced m_descriptor_length to the length of the entries.
        getline(ifs,sLine);
        std::istringstream ssD(sLine); std::istream_iterator<float> istD (ssD);
        unsigned int nVectorLength =  0;
        while(istD != std::istream_iterator<float>()) //Default constructor yields the end-of-stream iterator
        {
            istD++;
            nVectorLength++;
        }
        //Sanity check - vector should be at least as long as announced dimensions
        if(nVectorLength < m_descriptor_length)
        {
            MOOSTrace("Error: Descriptor with missing entries?. %d instead of %d\n", nVectorLength, m_descriptor_length);
        }
        //Otherwise, record how long the prepended image co-ord part is. We'll skip this bit when reading
        unsigned int nCoordsLength = nVectorLength - m_descriptor_length;
     
        //Now, read in all the descriptors
        PatchDescriptor Feature(nCoordsLength,nDim);
        for(int i = 0;i<nNumDescriptors;i++)
        {
            ssD.clear(); ssD.str(sLine);        //Put the string in the stringstream
            istream_iterator<float> isD (ssD);
            //Read in the image coords part
            for(unsigned int k=0;k<nCoordsLength;++k)
            {
                Feature.RegionDescriptor[k] = *isD++;
            }
            //Read in the actual descriptor
            std::vector<float> T;
            T.reserve(m_descriptor_length);
            std::copy(isD,std::istream_iterator<float>(),std::back_inserter(T));
            std::copy(T.begin(),T.end(),Feature.FeatureDescriptor.data_block());
            ReturnedDescriptors.push_back(Feature);

            getline(ifs,sLine);   //Grab the next line from the file
        }
    }
    ifs.close();
}

void CWordMaker::SetDescriptorLengthFromFeatureFile(const string &sFile)
{
    ifstream ifs(sFile.c_str());
    if(!ifs)
        MOOSTrace("Failed to open %s for reading\n", sFile.c_str());

    //read dimensions of the descriptors, and record it
    string sLine;
    getline(ifs,sLine);        
    m_descriptor_length = atoi(sLine.c_str());
}

void CWordMaker::WriteOXSLine(ofstream & oxs, const vector<int>& ReturnedWords,const BagOfRegions &RegionBag, const std::string &sImageName) const
{
    oxs << "SCENE:" << endl
        << sImageName << endl << endl;

    int vocab_size = m_Vocab.size();
    //Which words
    for(int i=0;i<vocab_size;++i)
    {
        if(ReturnedWords[i] != 0)
        {
            oxs << i << " ";
        }
    }
    oxs << endl;
    //How often
    for(int i=0;i<vocab_size;++i)
    {
        if(ReturnedWords[i] != 0)
        {
            oxs << ReturnedWords[i] << " ";
        }
    }
    oxs << endl;

    BagOfRegions::const_iterator pos;
    //Image part
    for(pos = RegionBag.begin(); pos != RegionBag.end();++pos)
    {
        oxs << pos->second.nPartID << " ";
    }
    oxs << endl;
    //Blob response
    for(pos = RegionBag.begin(); pos != RegionBag.end();++pos)
    {
        oxs << pos->second.strength << " ";
    }
    oxs << endl;
    //X
    for(pos = RegionBag.begin(); pos != RegionBag.end();++pos)
    {
        oxs << pos->second.x << " ";
    }
    oxs << endl;
    //Y
    for(pos = RegionBag.begin(); pos != RegionBag.end();++pos)
    {
        oxs << pos->second.y << " ";
    }
    oxs << endl;
    //1/r^2
    for(pos = RegionBag.begin(); pos != RegionBag.end();++pos)
    {
        oxs << pos->second.inv_scale << " ";
    }
    oxs << endl;
}
// For saving the SURF descriptors to an ASCII file
void CWordMaker::saveIpoints(const string &sFileName, const vector< InterestPoint >& ipts_view) const
{
  ofstream ipfile(sFileName.c_str());
  if( !ipfile ) {
    cerr << "ERROR saving descriptors: "
         << "Couldn't open file '" << sFileName.c_str() << "'!" << endl;
    return;
  }

  double sc;
  unsigned count = ipts_view.size();

  // Write the file header
    ipfile << m_descriptor_length << endl << count << endl;

  // Save interest point with descriptor in the format of Krystian Mikolajczyk.
  for (unsigned n=0; n<ipts_view.size(); n++){
    // circular regions with diameter 5 x scale
    sc = 2.5 * ipts_view[n].scale; sc*=sc;
    ipfile  << ipts_view[n].strength /* blob response */
            << " " << ipts_view[n].x /* x-location of the interest point */
            << " " << ipts_view[n].y /* y-location of the interest point */
            << " " << 1.0/sc /* 1/r^2 */
            << " " << 0.0     // For elliptical regions. SURFs are circular.
            << " " << 1.0/sc; /* 1/r^2 */

    // Here comes the descriptor
    for (unsigned int i = 0; i < ipts_view[n].descriptor.size(); i++) {
      ipfile << " " << ipts_view[n].descriptor[i];
    }
    ipfile << endl;
  }
  //Tidy up
  ipfile.close();
}


bool NotImage::operator () (const string &sFilename)
{
    string sExt= sFilename.substr(sFilename.rfind(".")+1);
    return !((sExt == "jpeg") || (sExt == "jpg") || (sExt == "pgm") || (sExt == "pnm") || (sExt == "ppm") || (sExt == "png") || (sExt == "tiff") || (sExt == "tif"));
}

bool NotDescriptor::operator () (const string &sFilename)
{
    string sExt= sFilename.substr(sFilename.rfind(".")+1);
    return !((sExt == "sift") || (sExt == "surf") || (sExt == "dat") || (sExt == "siftgeo"));
}

unsigned int CWordMaker::FindNumImagesPerScene(const list<string> &sContents) const
{
    //sContents is a sorted list of the image filenames from the image directory
    //For some sensors (e.g. stereo cameras and Ladybug), a single capture consists of multiple images
    //Our standard name format is
    //(stem)_(time)-(part_num)
    //where part_num identifies which CCD the image came from
    //e.g.
    //grab_1193766403.514-0.jpg
    //grab_1193766403.514-1.jpg
    //grab_1193766404.724-0.jpg
    //grab_1193766404.724-1.jpg
    //If captures have one part only, the -(part) suffix may be omitted.
    //This function looks at the list and decides how many images each capture/scene consists of.

    string sFilename = sContents.front();
    string sName= sFilename.substr(0,sFilename.rfind(".")); //strip the extension
    //Check for a -(part) suffix.
    if(sName.rfind("-") == std::string::npos)
    {
        return 1; //filename has no -(part) suffix. Captures have only one part
    }

    //We have -(part) suffixes. Check how many parts.
    unsigned int nParts = 1;
    std::list<std::string>::const_iterator p;
    p = sContents.begin();
    
    sFilename = *p++;
    string sFirstPartNum = sFilename.substr(sFilename.rfind("-")+1,1);

    while(p!=sContents.end())
    {
        sFilename = *p;
        string sPartNum = sFilename.substr(sFilename.rfind("-")+1,1);

        if(sPartNum == sFirstPartNum)
        {
            break;
        }
        else
        {
            nParts++;
        }
        ++p;
    }

    return nParts;
}
