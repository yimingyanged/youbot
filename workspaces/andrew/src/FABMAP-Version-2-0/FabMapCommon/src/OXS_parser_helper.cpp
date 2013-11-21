#include "OXS_parser_helper.h"
#include <MOOSLIB/MOOSLib.h>

vector<vector<ImageRegion> > LoadExcludedRegions(string sExcludedRegionsFilePath)
{
    vector<vector<ImageRegion> > ExcludedRegions;
    ExcludedRegions.reserve(6);

    if(sExcludedRegionsFilePath != "")
    {
        CProcessConfigReader fReader;
        fReader.SetFile(sExcludedRegionsFilePath);
        fReader.SetAppName("ExcludedRegions");
        if(!fReader.IsOpen())
        {
            cerr << "Error: Failed to open excluded regions definition file:" << endl << sExcludedRegionsFilePath << endl;
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
                    cerr << "Config error - badly formatted ExcludedRegion. Size was " << output.size() << endl;
                unsigned int imageID = (unsigned int) output[0];
                unsigned int sz = ExcludedRegions.size();
                if(ExcludedRegions.size() < (imageID+1))
                    ExcludedRegions.resize(imageID+1);
                ImageRegion newRegion;
                newRegion.x_low =  (float) output[1];
                newRegion.x_high = (float) output[2];
                newRegion.y_low =  (float) output[3];
                newRegion.y_high = (float) output[4];
                if(newRegion.x_low > newRegion.x_high || newRegion.y_low > newRegion.y_high)
                    cerr << "Error: Bad specification for excluded region. One rectangle dimension has zero size" << endl;
                ExcludedRegions[imageID].push_back(newRegion);
            }
        }
    }

    return ExcludedRegions;
}
