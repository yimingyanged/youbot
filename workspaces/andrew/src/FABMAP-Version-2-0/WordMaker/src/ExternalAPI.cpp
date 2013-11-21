#include "WordMaker.h"
#include <set>

//Set SURF threshold
void CWordMaker::SetSURFThreshold(const double SURF_thres)
{
    m_SURF_thres = SURF_thres;
}
//Set other SURF parameters externally
//SURF_upright and SURF128 cannot be safely changed without changing the vocabulary also
void CWordMaker::SetSURFOptions(const bool SURF_upright, const bool bSURF128)
{
    m_SURF_upright = SURF_upright;
    m_SURF_extended = bSURF128;
    m_descriptor_length = m_SURF_extended ? 128 : 64;
}

//Setup function for using WordMaker. Specify the vocabulary file used for feature quantization.
bool CWordMaker::LoadVocabularyAndSetup(const string &sVocabPath,
                                        const string &sVocabName,
                                        const unsigned int nDescriptorDimensionality)
{
    //VocabPath:                    the full path to the vocabulary
    //VocabName:                    the name of the vocabulary (for "Foo.oxv", supply "Foo")
    //nDescriptorDimensionality:    the dimensionality of the descriptors to be processed

    m_sVocabPath = EnsurePathHasTrailingSlash(sVocabPath);
    m_sVocabName = sVocabName;
    m_descriptor_length = nDescriptorDimensionality;

    if(!InitializeKDTree())
    {
        cerr << "WARNING - Failed to create kdTree" << endl;
        return false;
    }
    else
    {
        return true;
    }
}

//Given a vector of feature vectors, return a bag-of-words.
bool CWordMaker::QuantizeFeatures(vector<vector<float> > &FeatureVectors,
                                  vector<int> &ReturnedWords)
{
    vector<int> ReturnedWords_binary(m_Vocab.size(),0);

    unsigned int NF = FeatureVectors.size();
    for(unsigned int iF = 0; iF < NF; iF++)
    {
        //Make sure the data is valid
        if(FeatureVectors[iF].size() != m_descriptor_length)
        {
            cerr << "Error! Word maker received a descriptor of an unexpected length" << endl
                 << "  Expected dimensionality: " << m_descriptor_length << " Supplied descriptor dimensionality: " << FeatureVectors[iF].size() << endl;
            return false;
        }
        //Submit the query
        vector<int> NearNeighbourIndices(1);
        FindNeighborsFloat(&(*NearNeighbourIndices.begin()),
            NearNeighbourIndices.size(),
            &(*FeatureVectors[iF].begin()),
            m_LNNIndex,
            m_kDTreeNodesToCheck);

        int nWord = NearNeighbourIndices[0];

        //Record the existence of the word
        ReturnedWords_binary.at(nWord) += 1;
    }

    //Change from vector format [0 1 0  0 1...] to [2 5 ...]
    ConvertReturnedWordsFromBinaryToIndexFormat(ReturnedWords_binary,ReturnedWords);

    return true;
}

// Compute the bag-of-words for the image at the specified path.
bool CWordMaker::GetBagOfWordsForImage(const string &sImagePath_andFilename,
                                       vector<int> &ReturnedWords)
{
    //User doesn't care about interest point geometry.
    BagOfRegions discard;
    return GetBagOfWordsForImage(sImagePath_andFilename,ReturnedWords,discard);
}

// Compute the bag-of-words for the image at the specified path
// Return both the bag-of-words and the interest point geometry.
bool CWordMaker::GetBagOfWordsForImage(const string &sImagePath_andFilename,
                                       vector<int> &ReturnedWords,
                                       BagOfRegions &RegionBag)
{
  /*  CImg<double> image = CImg<>(sImagePath_andFilename.c_str());

    //Convert to greyscale
    //The correct way to do this is via
    //const CImg<double> dest = image.get_RGBtoYCbCr().channel(0);
    //Which extracts the luminance channel from YCbCr
    //The way here is 10x faster, so we prefer it for online usage.
    //However, still to be verified if it impacts recognition performance.
    image.resize(-100,-100,-100,1);

    //Copy data into a SURF image object.
    //This is somewhat slow, possibly worth optimizing this somehow.
    const double normalizer = 255.00001;
    Image *im = new Image(image.width(),image.height());
    for (unsigned int y = 0; y < image.height(); y++)
        for (unsigned int x = 0; x < image.width(); x++)
            im->setPix(x, y, image(x,y)/normalizer);
*/
	/*  IplImage *source = cvLoadImage(sImagePath_andFilename.c_str());

	                         // Here we retrieve a percentage value of source size
	                         int percent = 100; // use 100% as old code

	                         // declare a destination IplImage object with correct size, depth and channels
	                         IplImage *image = cvCreateImage( cvSize((int)((source->width*percent)/100) , (int)((source->height*percent)/100) ), source->depth, source->nChannels );

	                         //use cvResize to resize source to a destination image
	                         cvResize(source, image);

*/


    //Convert to bag of words.
    const unsigned int nDummyPartID=0;
    const string sDummyName = "aName";
    string fileNamePath = sImagePath_andFilename;
    vector<int> ReturnedWords_binary_vector_format(m_Vocab.size(),0);
    ProduceBagOfWords(fileNamePath, sDummyName, ReturnedWords_binary_vector_format,
                      RegionBag, nDummyPartID);

    //Change from [0 1 0 0 1...] to [2 5 ...]
    ConvertReturnedWordsFromBinaryToIndexFormat(ReturnedWords_binary_vector_format,
                                                ReturnedWords);

    //Tidy up
 //   cvReleaseImage(&image);
//    delete image;
    return true;
}

//***************************
// Private helper functions
//***************************

void CWordMaker::ConvertReturnedWordsFromBinaryToIndexFormat(const vector<int> &ReturnedWords_Binary,
                                                             vector<int> &ReturnedWords_Index)
{
    //Converts words in the format [0 1 0 0 1...] (i.e vector, length of vocab, with counts)
    //to the format [2 5 ...] (i.e. indices of which words appear)
    //Note that count info is lost.

    ReturnedWords_Index.clear();
    int vocab_size = ReturnedWords_Binary.size();
    for(int i=0;i<vocab_size;++i)
    {
        if(ReturnedWords_Binary[i] != 0)
        {
            ReturnedWords_Index.push_back(i);
        }
    }
}
