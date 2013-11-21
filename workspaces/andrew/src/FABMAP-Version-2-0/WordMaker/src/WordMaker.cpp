#include "WordMaker.h"
#include <set>

CWordMaker::CWordMaker()
{
    string m_sBaseDataPath = "";
    string m_sDescriptorPath = "";
    string m_sWordOutputPath = "";
    bool m_bWriteWords = false;
    bool m_bWriteDescriptors = false;

    bFormatINRIA = false;

    ExcludedRegions.reserve(6);

    //KdTree
    m_kDTreeNodesToCheck = 512;
    m_nNumkDTrees = 8;
    
    //SURF parameters
    //These first few you might want to tinker with
    m_SURF_thres = 25.0;            //Yields a ~100 regions on typical images
    m_SURF_doubleImageSize = false;
    m_SURF_upright = true;
    m_SURF_extended = true;
    //The rest are probably best left at defaults.
    m_SURF_initLobe = 3;
    m_SURF_samplingStep = 2;
    m_SURF_octaves = 4;
    m_SURF_indexSize = 4;
}


CWordMaker::~CWordMaker() {}

void CWordMaker::DoInitialSetup()
{
    //Read Params from mission file
    //SURF Params
    m_MissionReader.GetConfigurationParam("SURFThres", m_SURF_thres);
    m_MissionReader.GetConfigurationParam("SURFUpright", m_SURF_upright);
    m_MissionReader.GetConfigurationParam("SURF128", m_SURF_extended);
    m_descriptor_length = m_SURF_extended ? 128 : 64;
    
    //Data file format
    string sDataFormat;
    m_MissionReader.GetConfigurationParam("DataFormat", sDataFormat);
    if(sDataFormat == "INRIA")
        bFormatINRIA = true;

    //Where is the vocabulary
    m_MissionReader.GetConfigurationParam("VocabPath", m_sVocabPath);
    m_MissionReader.GetConfigurationParam("VocabName", m_sVocabName);
    m_sVocabPath = EnsurePathHasTrailingSlash(m_sVocabPath);

    //Base data path. This is where we output .words and .surf files, and read in images (if in Batch mode) 
    m_MissionReader.GetConfigurationParam("BaseDataPath", m_sBaseDataPath);
    m_sBaseDataPath = EnsurePathHasTrailingSlash(m_sBaseDataPath);

    //In Batch mode - Where are the Images?
    m_sImagePath = m_sBaseDataPath + "Images/";
    m_MissionReader.GetConfigurationParam("ImagePath", m_sImagePath);
    m_sImagePath = EnsurePathHasTrailingSlash(m_sImagePath);

    //Do we want to write .surf and .words files
    //And where do we want to put them?
    m_MissionReader.GetConfigurationParam("WriteWords", m_bWriteWords);
    //Set default output path
    m_sWordOutputPath = m_sBaseDataPath + "Words_" + m_sVocabName + "/";
    //Read in a custom path if there is one
    m_MissionReader.GetConfigurationParam("WordOutputPath", m_sWordOutputPath);

    //Same for descriptors
    m_MissionReader.GetConfigurationParam("WriteDescriptors", m_bWriteDescriptors);
    if(!bFormatINRIA)
        m_sDescriptorPath = m_sBaseDataPath + "Surf/";
    else
        m_sDescriptorPath = m_sBaseDataPath + "siftgeo/";
    m_MissionReader.GetConfigurationParam("DescriptorPath", m_sDescriptorPath);

    //Make directories if necessary
    if(m_bWriteWords)
    {
        m_sWordOutputPath = EnsurePathHasTrailingSlash(m_sWordOutputPath);
        MOOSCreateDirectory(m_sWordOutputPath);	
    }

    if(m_bWriteDescriptors)
    {
        m_sDescriptorPath = EnsurePathHasTrailingSlash(m_sDescriptorPath);
        MOOSCreateDirectory(m_sDescriptorPath);	
    }

    //Excluded regions
    //Optionally define regions of the image where detected features should be discarded.
    //e.g. To mask out parts of the robot visible in the camera image.
    string sExcludedRegionsFilePath;
    m_MissionReader.GetConfigurationParam("ExcludedRegionsDefinitionFile",sExcludedRegionsFilePath);
    if(sExcludedRegionsFilePath != "")
    {
        CProcessConfigReader fReader;
        fReader.SetFile(sExcludedRegionsFilePath);
        fReader.SetAppName("ExcludedRegions");
        if(!fReader.IsOpen())
        {
            MOOSTrace("Error: Failed to open excluded regions definition file:\n%s",sExcludedRegionsFilePath.c_str());
        }
        else
        {
            string sExcludedRegions;
            fReader.GetConfigurationParam("ExcludedRegions",sExcludedRegions);
            while(sExcludedRegions.size() > 1)
            {
                vector<double> output; int r,c;
                MOOSVectorFromString(sExcludedRegions,output,r,c);
                MOOSChomp(sExcludedRegions,";");
                if(output.size() != 5)
                    MOOSTrace("Config error - badly formatted ExcludedRegion. Size was %d",output.size());
                unsigned int imageID = (unsigned int) output[0];
                unsigned int sz = ExcludedRegions.size();
                if(ExcludedRegions.size() < (imageID+1))
                    ExcludedRegions.resize(imageID+1);
                ImageRegion newRegion;
                newRegion.x_low =  output[1];
                newRegion.x_high = output[2];
                newRegion.y_low =  output[3];
                newRegion.y_high = output[4];
                if(newRegion.x_low > newRegion.x_high || newRegion.y_low > newRegion.y_high)
                    MOOSTrace("Error: Bad specification for excluded region. One rectangle dimension has zero size\n");
                ExcludedRegions[imageID].push_back(newRegion);
            }
        }
    }

    //Mode
     string sMode;
     m_MissionReader.GetConfigurationParam("Mode", sMode);
     MOOSToUpper(sMode);

    //Load voacbulary and setup kD-tree
     if(sMode != "SURF_ONLY")
     {
        if(!InitializeKDTree())
            MOOSTrace("WARNING - Failed to create kdTree");
     }

}

bool CWordMaker::InitializeKDTree()
{
    //Read in the vocabulary file
    if(! ReadVocabularyFile() )
        return false;

    //Setup the kd-tree
    int nTreeSize = m_Vocab.size();

    if(nTreeSize==0)
        return false;

    int nDim = m_Vocab[0].size();

    if (nDim != m_descriptor_length)
        MOOSFail("Yikes! Vocab dimensionality does not match SURF settings");

    m_LNNData.set_size(nTreeSize,nDim);

    //convert VNLdata to be in LNN format
    for(int i=0;i<nTreeSize;++i)
    {        
        const float * pD = m_Vocab[i].data_block();
        for(int j=0;j<nDim;++j)
            m_LNNData[i][j]=pD[j];
    }

    m_LNNIndex = BuildIndexFloat(m_LNNData.data_array(), nTreeSize, nDim, m_nNumkDTrees);

    //Done
    return true;
}

void CWordMaker::ProduceBagOfWords(string &filePath, const string &sImageName,
                                   vector<int> &ReturnedWords) const
{
    //Create dummy variables for geometry information, user doesn't want it
    BagOfRegions discard;
    unsigned int nDummyID=0;
    ProduceBagOfWords(filePath,sImageName,ReturnedWords,discard,nDummyID);
}

void CWordMaker::ProduceBagOfWords(string &filePath, const string &sImageName,
                                   vector<int> &ReturnedWords,
                                   BagOfRegions &RegionBag,
                                   const unsigned int nCurrentPartID) const
{
    std::vector< SurfInterface::InterestPoint > ipts;

    // upright true by default
    SurfTools::computeInterestPoints(filePath,ipts, m_SURF_octaves, m_SURF_indexSize, m_SURF_thres, m_SURF_extended);
   //cout << "here3" << endl;

       //       m_SURF_thres, m_SURF_doubleImageSize,
         //       m_SURF_initLobe, m_SURF_samplingStep, m_SURF_octaves,
        //        m_SURF_upright, m_SURF_extended, m_SURF_indexSize);


    //The Surf library doesn't define a correct copy constructor for KeyPoint (it doesn't do a deep copy of the data pointer)
    //Also, Ipoint doesn't know what size its descriptor is, so we can't define a correct copy constructor ourselves.
    //The upshot of this is that we can't apply modifying algorithms to collections of Ipoint
    //Instead, we'll have to view the Ipoint vector through a layer of indirection, and apply the modifications to the view.

   /* vector<libsurf::KeyPoint*> ipts_view;
    ipts_view.reserve(ipts.size());
    for(unsigned int i=0;i<ipts.size();++i)
    {
        ipts_view.push_back(&(ipts[i]));
    }
*/
    //Discard any interest points that lie in user-defined excluded regions of the image
    //Used, for example, to blank out parts of the robot visible in the camera frame.
    DiscardIptsInExcludedRegions(ipts,nCurrentPartID);
    
    //Optionally save the desciptors
    if(m_bWriteDescriptors)
    {
        saveIpoints(m_sDescriptorPath + sImageName + ".surf" , ipts);
    }

    //Quantize the descriptors to words.
  //  cout << "here4" << endl;

    QuantizeDescriptors(ipts,sImageName,ReturnedWords,RegionBag,nCurrentPartID);
  //  cout << "here5" << endl;

}


//Functor for removing interest points
class IptIsInRegion
{
public:
    IptIsInRegion (ImageRegion r) : region(r) {}

    bool operator () (const SurfInterface::InterestPoint ipoint)
    {
        return (ipoint.x > region.x_low && ipoint.x < region.x_high) && (ipoint.y > region.y_low && ipoint.y < region.y_high);
    }
private:
    ImageRegion region;
};

//Discard any interest points that lie within excluded regions of the image
//nCurrentPartID is to specify which camera image in a camera with multiple CCDs, e.g. Stereo or Ladybug.
void CWordMaker::DiscardIptsInExcludedRegions(vector<SurfInterface::InterestPoint>& ipts_view,
                                              const unsigned int nCurrentPartID) const
{
    if(nCurrentPartID < ExcludedRegions.size())
    {
        for(unsigned int r=0;r<ExcludedRegions[nCurrentPartID].size();++r)
        {
            ImageRegion region = ExcludedRegions[nCurrentPartID][r];
            ipts_view.erase(remove_if(ipts_view.begin(),ipts_view.end(),IptIsInRegion(region)),ipts_view.end());
        }
    }
}

void CWordMaker::QuantizeDescriptors(const vector< SurfInterface::InterestPoint >& ipts_view,
                                     const string &sImageName,
                                     vector<int> &ReturnedWords,
                                     BagOfRegions &RegionBag,
                                     const unsigned int nCurrentPartID) const
{
    //Change to PatchDescriptors
    int NF = ipts_view.size();
    vector<PatchDescriptor> Descriptors;
    PatchDescriptor SingleDescriptor(6,m_descriptor_length);
    
    for(int iF = 0;iF < NF;iF++)
    {    
        //Actual Descriptor
        for(unsigned int i=0;i< ipts_view[iF].descriptor.size();++i)
        {

            SingleDescriptor.FeatureDescriptor[i] = (float) ipts_view[iF].descriptor[i]; // openSURF changed to 64
        }
        //Image Region info
        //Circular regions with diameter 5 x scale
        double sc = 2.5 * ipts_view[iF].scale; sc*=sc;
        SingleDescriptor.RegionDescriptor[0] = ipts_view[iF].strength;   // blob response of the interest point
        SingleDescriptor.RegionDescriptor[1] = ipts_view[iF].x;          // x-location of the interest point
        SingleDescriptor.RegionDescriptor[2] = ipts_view[iF].y;          // y-location of the interest point
        SingleDescriptor.RegionDescriptor[3] = 1.0/sc;                    // 1/r^2
        SingleDescriptor.RegionDescriptor[4] = 0.0;                       // For elliptical regions. 0 for circular regions.
        SingleDescriptor.RegionDescriptor[5] = 1.0/sc;                    // 1/r^2
        //Store it
        Descriptors.push_back(SingleDescriptor);
    }

    //Look up in kd-tree
    QuantizeDescriptors(Descriptors,sImageName,ReturnedWords,RegionBag,nCurrentPartID);
}

void CWordMaker::QuantizeDescriptors(const vector<PatchDescriptor>& Descriptors,
                                     const string &sImageName,
                                     vector<int> &ReturnedWords,
                                     BagOfRegions &RegionBag,
                                     const unsigned int nCurrentPartID) const
{
    int NF = Descriptors.size();

    //Optionally save the bag of words
    std::ofstream of;
    if(m_bWriteWords)
    {
        //Open the file for writing
        std::string sFilePath = m_sWordOutputPath + sImageName + ".words";
        of.open(sFilePath.c_str());
        if(!of)
            MOOSTrace("Failed to open %s for writing\n",sFilePath.c_str());
        of << m_sVocabName << std::endl;//Record what vocab these words relate to
    }

    vnl_vector<float> Query;
    for(int iF = 0;iF < NF;iF++)
    {    
        Query = Descriptors[iF].FeatureDescriptor;
        //Submit the query
         vector<int> NearNeighbourIndices(1);
        FindNeighborsFloat(&(*NearNeighbourIndices.begin()),
            NearNeighbourIndices.size(),
            &(*Query.begin()),
            m_LNNIndex,
            m_kDTreeNodesToCheck);
        
        int nWord = NearNeighbourIndices[0];

        //How far away is the word centre from the query point?
        vnl_vector<float> WordCentre = m_Vocab.at(nWord);
        double dfDistance = (Query - WordCentre).two_norm();

        // You can optionally insert a distance cut-off for quantization here.
        // e.g. Originally we tested (dfDistance<1.5*m_dfClusterThreshold), where
        // dfClusterThreshold was the clustering threshold used in an incremental
        // radius-based clustering. With KMeans there is no such thing as a
        // m_dfClusterThreshold, so I have turned this off.
        if(true)
        {
            //Record the existence of the word
            ReturnedWords.at(nWord) += 1;
            PatchLocation patch_location(nCurrentPartID,Descriptors[iF].RegionDescriptor);
            RegionBag.insert(make_pair(nWord,patch_location));
        
            if(m_bWriteWords)
            {
                //Output the PatchDescriptor <-> Word association.
                //This is for visualizing which regions get mapped to which words.
                of  << patch_location.strength   // strength of the interest point
                    << " " << patch_location.x   // x-location of the interest point
                    << " " << patch_location.y   // y-location of the interest point
                    << " " << patch_location.inv_scale   // 1/r^2
                    << " " << patch_location.skew        // For elliptical regions.
                    << " " << patch_location.inv_scale;  // 1/r^2
                //Word ID
                of << " " <<  nWord << std::endl;
            }
        }
        else
        {
            //This is the space for fancy things like online vocabulary learning.
            //But nothing implemented yet.
            MOOSTrace("Saw a feature further than 1.5 times cluster threshold of any known word\n");
        }   
    }
    
    if(m_bWriteWords)
    {
        of.close();
    }
}

void CWordMaker::BatchProcess(string sAppName, string sMissionFile)
{    //Function to process all the images in a directory, and output .surfs, .words, or .oxs
    double dfStartTime = HPMOOSTime();

    //Read in config params
    m_MissionReader.SetAppName(sAppName);
   if(!m_MissionReader.SetFile(sMissionFile.c_str()))
   {
        MOOSTrace("Warning Mission File \"%s\" not found...\n",sMissionFile.c_str());
   }
    
    DoInitialSetup();
        
    bool bOXSOutput = false;
    m_MissionReader.GetConfigurationParam("WriteOXS", bOXSOutput);

    if(!(m_bWriteWords || m_bWriteDescriptors || bOXSOutput))
    {
        MOOSTrace("\nWARNING - BatchProcess called without any outputs set!\n"); 
    }

    //Get a list of images in the directory
    list<string> sContents;
    string sFilename;
    if(GetDirectoryContents(m_sImagePath,sContents))
    {
        sContents.remove_if( NotImage() );
        sContents.sort();

        unsigned int nNumImagesPerScene = FindNumImagesPerScene(sContents);
        MOOSTrace("Found %d images to process. There are %d images per scene. \nImages processed...",sContents.size(),nNumImagesPerScene);

        //If we're going to write the .oxs file, setup an output
        std::ofstream oxs;
        if(bOXSOutput)
        {
            //string sVocabName = m_sVocabFile.substr(m_sVocabFile.rfind("/")+1,m_sVocabFile.size()-m_sVocabFile.rfind("/")-5);
            std::string sOXSFilePath = m_sBaseDataPath + "Scenes_" + m_sVocabName + ".oxs";
            oxs.open(sOXSFilePath.c_str());
            if(!oxs)
                MOOSTrace("Failed to open %s for writing\n",sOXSFilePath.c_str());
            //Write header
            oxs << "VOCABULARY:" << m_sVocabName << endl;
            oxs << "SCENES:" << (sContents.size() / nNumImagesPerScene)  << endl;
            oxs << "WORDS:" << m_Vocab.size() << endl;
        }

        //Process directory contents
        int counter = 0;   
        while(!sContents.empty())
        {
            vector<int> ReturnedWords(m_Vocab.size());
            BagOfRegions RegionBag;
            bool bCompleteGrab = true;  //For dealing with multi-part images such as stereo or Ladybug which consist of multiple individual image files.
                                        // We expect these files to follow the naming convention imagefoo-0.jpg, imagefoo-1.jpg, imagebar-0.jpg, imagebar-1.jpg, ...

            for(unsigned int i=0;i<nNumImagesPerScene;i++)
            {
                if(sContents.empty())
                {   //Sometimes if the capture process is aborted, the final grab may be incomplete. Discard this grab.
                    bCompleteGrab = false;
                    break;
                }

                sFilename = sContents.front();
                sContents.pop_front();
                string sName= sFilename.substr(0,sFilename.rfind("."));

                //For multi-part grabs, make sure we're processing the right part number (i.e. image-0.jpg, image-1.jpg)
                if(nNumImagesPerScene>1)
                {
                    if(atoi((sName.substr(sName.rfind("-")+1,1)).c_str()) != i)
                    {
                        MOOSTrace("\n ERROR - Processing an unexpected part of a multipart capture. DOOOM!\n");
                        bCompleteGrab = false;
                    }
                }

                //Load it
              // CImg<double> image = CImg<>((m_sImagePath+sFilename).c_str());
                 //Convert to greyscale
                //The correct way to do this is via
                //const CImg<double> dest = image.get_RGBtoYCbCr().channel(0);    
                //Which extracts the luminance channel from YCbCr
                //This resize way is 10x faster, so we prefer it for online usage.
                //However, still to be verified if it impacts recognition performance.

                //image.resize(-100,-100,-100,1);



              //  IplImage *source = cvLoadImage((m_sImagePath+sFilename).c_str());

                // Here we retrieve a percentage value of source size
            //    int percent = 100; // use 100% as old code

                // declare a destination IplImage object with correct size, depth and channels
             //   IplImage *image = cvCreateImage( cvSize((int)((source->width*percent)/100) , (int)((source->height*percent)/100) ), source->depth, source->nChannels );

                //use cvResize to resize source to a destination image
           //     cvResize(source, image);

                string fileNamePath = m_sImagePath+sFilename;
           //     cout << "HERE1" << endl;
                ProduceBagOfWords(fileNamePath,sName, ReturnedWords, RegionBag, i);
             //   cout << "HERE2" << endl;
                             //Tidy up
                           //  cvReleaseImage(&image);
                         //    delete image;





                //Copy data into a SURF image object
              /*  const double normalizer = 255.00001;
                Image *im = new Image(image.width(),image.height());
                for (unsigned int y = 0; y < image.height(); y++)
                    for (unsigned int x = 0; x < image.width(); x++)
                        im->setPix(x, y, image(x,y)/normalizer);
*/

                //Convert to bag of words
            //    ProduceBagOfWords(im, sName, ReturnedWords, RegionBag, i);

                //Progress
                if(++counter % 10 == 0)
                {
                    MOOSTrace("%d...",counter);
                }
            }
            //Now, output to .oxs
            if(bOXSOutput && bCompleteGrab)
            {
                WriteOXSLine(oxs,ReturnedWords,RegionBag,m_sImagePath+sFilename);
            }      
        }
        //Tidy up
        if(bOXSOutput)
         oxs.close();
    }

    //Finish
    MOOSTrace("Done");
    double timeTaken = HPMOOSTime() - dfStartTime;
    if(timeTaken > 60.0)
    {
        cout << endl << "Total time " << setprecision(3) << ((timeTaken - fmod(timeTaken,60.0))/60.0) << " minutes " 
            << fmod(timeTaken,60.0) << " seconds." << endl;
    }
    else
    {
        cout << endl << "Total time " << timeTaken  << " seconds." << endl;
    }
}

void CWordMaker::GenerateSURFsOnly(string sAppName, string sMissionFile)
{    //Function to process all the images in a directory, and output .surfs
     //This is for use before we have a vocabulary available.
    double dfStartTime = HPMOOSTime();

    //Read in config params
    m_MissionReader.SetAppName(sAppName);
   if(!m_MissionReader.SetFile(sMissionFile.c_str()))
   {
        MOOSTrace("Warning Mission File \"%s\" not found...\n",sMissionFile.c_str());
   }
    
    DoInitialSetup();    
    
    //Get a list of images in the directory
    list<string> sContents;
    string sFilename;
    if(GetDirectoryContents(m_sImagePath,sContents))
    {
        sContents.remove_if( NotImage() );

        unsigned int nNumImagesPerScene = FindNumImagesPerScene(sContents);
        MOOSTrace("Found %d images to process. There are %d images per scene. \nImages processed...",sContents.size(),nNumImagesPerScene);
        
        //Process directory contents
        int counter = 0;
        while(!sContents.empty())
        {
            sFilename = sContents.back();
            sContents.pop_back();

            //Check it's an image
            string sName= sFilename.substr(0,sFilename.rfind("."));

            //For multi-part images, which part is it?
            int nCurrentImagePart = 0;
            if(nNumImagesPerScene>1)
                nCurrentImagePart = atoi((sName.substr(sName.rfind("-")+1,1)).c_str());

         /*   //Load it
            CImg<double> image = CImg<>((m_sImagePath+sFilename).c_str());

            //Convert to greyscale
            image.resize(-100,-100,-100,1); 

            //Create a SURF image object
            const double normalizer = 255.00001;
            Image *im = new Image(image.width(),image.height());
            for (unsigned int y = 0; y < image.height(); y++)
                for (unsigned int x = 0; x < image.width(); x++)
                    im->setPix(x, y, image(x,y)/normalizer);
*/
/*
            IplImage *source = cvLoadImage((m_sImagePath+sFilename).c_str());

                         // Here we retrieve a percentage value of source size
                         int percent = 100; // use 100% as old code

                         // declare a destination IplImage object with correct size, depth and channels
                         IplImage *image = cvCreateImage( cvSize((int)((source->width*percent)/100) , (int)((source->height*percent)/100) ), source->depth, source->nChannels );

                         //use cvResize to resize source to a destination image
                         cvResize(source, image);
*/


            std::vector< SurfInterface::InterestPoint > ipts;
            string fileNamePath =m_sImagePath+sFilename;
            SurfTools::computeInterestPoints(fileNamePath,ipts, m_SURF_octaves, m_SURF_indexSize, m_SURF_thres, m_SURF_extended);

            //Compute the SURF descriptor
          /*  surfDetDes(    image,
               		ipts,
               		m_SURF_upright,
               		m_SURF_octaves,
               		4,
               		m_SURF_samplingStep,
               		m_SURF_thres);
               		*/
            /* surfDetDes(    im, ipts,
                m_SURF_thres, m_SURF_doubleImageSize,
                m_SURF_initLobe, m_SURF_samplingStep, m_SURF_octaves,
                m_SURF_upright, m_SURF_extended, m_SURF_indexSize);
*/
            //Define a view of the ipoints, so that we can manipulate the collection
            //Need to do this because the SURF library doesn't define a correct copy constructor for KeyPoint.
           /* vector<SurfInterface::InterestPoint> ipts_view;
            ipts_view.reserve(ipts.size());
            for(unsigned int i=0;i<ipts.size();++i)
            {
                ipts_view.push_back(&(ipts[i]));
            }
            */
            DiscardIptsInExcludedRegions(ipts,nCurrentImagePart);

            saveIpoints(m_sDescriptorPath + sName + ".surf" , ipts);

            //Tidy up
                                          //    cvReleaseImage(&image);
                                         //     delete image;

            //Progress
            if(++counter % 10 == 0)
            {
                MOOSTrace("%d...",counter);
            }

        }
    }

    //Finish
    MOOSTrace("Done");
    double timeTaken = HPMOOSTime() - dfStartTime;
    if(timeTaken > 60.0)
    {
        cout << endl << "Total time " << setprecision(3) << ((timeTaken - fmod(timeTaken,60.0))/60.0) << " minutes " 
            << fmod(timeTaken,60.0) << " seconds." << endl;
    }
    else
    {
        cout << endl << "Total time " << timeTaken  << " seconds." << endl;
    }
}

void CWordMaker::QuantizeOnly(string sAppName, string sMissionFile)
{    //Function to process all the surf files in a directory, and output .words or .oxs
    double dfStartTime = HPMOOSTime();

    //Read in config params
    m_MissionReader.SetAppName(sAppName);
   if(!m_MissionReader.SetFile(sMissionFile.c_str()))
   {
        MOOSTrace("Warning Mission File \"%s\" not found...\n",sMissionFile.c_str());
   }
    
    DoInitialSetup();
        
    bool bOXSOutput = false;
    m_MissionReader.GetConfigurationParam("WriteOXS", bOXSOutput);

    if(!(m_bWriteWords || bOXSOutput))
    {
        MOOSTrace("\nWARNING - QuantizeOnly called without any outputs set!\n"); 
    }

    //Get a list of descriptors in the directory
    list<string> sContents;
    string sFilename;
    if(GetDirectoryContents(m_sDescriptorPath,sContents))
    {
        sContents.remove_if( NotDescriptor() );
        sContents.sort();

        MOOSTrace("Found %d images to process.",sContents.size());
        //Set the descriptor length and the number of images per scene
        unsigned int nNumImagesPerScene = FindNumImagesPerScene(sContents);
        SetDescriptorLengthFromFeatureFile(m_sDescriptorPath + sContents.front());
        MOOSTrace(" There are %d images per scene. \nImages processed...",nNumImagesPerScene);

 
        //If we're going to write the .oxs file, setup an output
        std::ofstream oxs;
        if(bOXSOutput)
        {
            std::string sOXSFilePath = m_sBaseDataPath + m_sVocabName + ".oxs";
            oxs.open(sOXSFilePath.c_str());
            if(!oxs)
                MOOSTrace("Failed to open %s for writing\n",sOXSFilePath.c_str());
            //Write header
            oxs << "VOCABULARY:" << m_sVocabName << endl;
            oxs << "SCENES:" << (sContents.size() / nNumImagesPerScene)  << endl;
            oxs << "WORDS:" << m_Vocab.size() << endl;
        }

        //Process directory contents
        int counter = 0;   
        while(!sContents.empty())
        {
            vector<int> ReturnedWords(m_Vocab.size());
            BagOfRegions RegionBag;

            bool bCompleteGrab = true;

            for(unsigned int i=0;i<nNumImagesPerScene;i++)
            {
                if(sContents.empty())
                {   //Sometimes if the capture process is aborted, the final grab may be incomplete. Discard this grab.
                    bCompleteGrab = false;
                    break;
                }

                sFilename = sContents.front();
                sContents.pop_front();
                string sName= sFilename.substr(0,sFilename.rfind("."));

                //For multi-part grabs, make sure we're processing the right part number (i.e. image-0.jpg, image-1.jpg)
                if(nNumImagesPerScene>1)
                {
                    if(atoi((sName.substr(sName.rfind("-")+1,1)).c_str()) != i)
                    {
                        MOOSTrace("\n ERROR - Processing an unexpected part of a multipart capture. DOOOM!\n");
                        bCompleteGrab = false;
                    }
                }

                //Read in the descriptor
                vector<PatchDescriptor> Descriptors;
                ReadFeatureFile(m_sDescriptorPath+sFilename, Descriptors);

                //Convert to bag of words
                QuantizeDescriptors(Descriptors, sName, ReturnedWords, RegionBag, i);
               
                //Progress
                if(++counter % 10 == 0)
                {
                    MOOSTrace("%d...",counter);
                }
            }
            //Now, output to .oxs
            if(bOXSOutput && bCompleteGrab)
            {
                WriteOXSLine(oxs,ReturnedWords,RegionBag,m_sImagePath+sFilename.substr(0,sFilename.rfind("."))+".jpg");
            }      
        }
        //Tidy up
        if(bOXSOutput)
         oxs.close();
    }

    //Finish
    MOOSTrace("Done");
    double timeTaken = HPMOOSTime() - dfStartTime;
    if(timeTaken > 60.0)
    {
        cout << endl << "Total time " << setprecision(3) << ((timeTaken - fmod(timeTaken,60.0))/60.0) << " minutes " 
            << fmod(timeTaken,60.0) << " seconds." << endl;
    }
    else
    {
        cout << endl << "Total time " << timeTaken  << " seconds." << endl;
    }
}
