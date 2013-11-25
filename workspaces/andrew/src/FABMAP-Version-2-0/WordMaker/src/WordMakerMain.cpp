#if (_MSC_VER == 1200)
#pragma warning(disable : 4503)
#pragma warning(disable : 4511)
#pragma warning(disable : 4512)
#endif

#include "WordMaker.h"
using namespace std;

//***********************************************************
//  WordMaker generates bag-of-word data from input images.
//  This stand-alone binary can be used from the command
//  line to process a directory of images.
//  WordMaker can also be used as a library from within 
//  your own C++ or Python program.
//***********************************************************

int main(int argc, char * argv[])
{
    const char * sMissionFile = "Config.moos";
    const char * sMOOSName    = "pWordMaker";
    
    switch(argc)
    {
    case 3:
        sMOOSName    = argv[2];
    case 2:
        sMissionFile = argv[1];
    }
    
    //Check what mode to run in.
    CMOOSFileReader m_FileReader;
    m_FileReader.SetFile(sMissionFile);
    string sMode;
    m_FileReader.GetValue("Mode", sMode);
    MOOSToUpper(sMode);

   CWordMaker word_maker;
    if(sMode == "BATCH")
    {
       // Process all the images in a directory.
       // Generates SURF feature files and bag-of-words data.
       word_maker.BatchProcess(sMOOSName,sMissionFile);
    }
    else if(sMode == "SURF_ONLY")
    {
       // Process all the images in a directory.
       // Generates SURF feature files only.
       word_maker.GenerateSURFsOnly(sMOOSName,sMissionFile);
    }
    else if(sMode == "QUANTIZE_ONLY")
    {
      // Process all the SURF feature files in a directory.
      // Generates bag-of-words data only.
      word_maker.QuantizeOnly(sMOOSName,sMissionFile);
    }
    
    return 0;
}
