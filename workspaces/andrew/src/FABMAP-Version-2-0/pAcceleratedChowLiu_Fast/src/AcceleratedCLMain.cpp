#include "AcceleratedCL.h"
#include "MOOSGenLib/ProcessConfigReader.h"
#include "IOHelperFunctions.h"

int main(int argc, char **argv) 
{    
    //Note to self
    //MARGINALS - For Ingmar, smooth them
    //Otherwise, turn off smoothing
    //See note in relevant function.

    const char * sMissionFile = "Config.moos";
    const char * sMOOSName    = "pAcceleratedCL";
    
    switch(argc)
    {
    case 3:
        sMOOSName    = argv[2];
    case 2:
        sMissionFile = argv[1];
    }
   
    //Find out where the data lives
    CProcessConfigReader MissionReader;
    MissionReader.SetFile(sMissionFile);
    MissionReader.SetAppName(sMOOSName);

    string datasetPath, datasetName, outputPath;
    MissionReader.GetConfigurationParam("DatasetPath", datasetPath);
    MissionReader.GetConfigurationParam("DatasetName", datasetName);
    MissionReader.GetConfigurationParam("OutputPath", outputPath);
    if(outputPath == "")
        outputPath = datasetPath;
    datasetPath = EnsurePathHasTrailingSlash(datasetPath);
    outputPath = EnsurePathHasTrailingSlash(outputPath);

    //Optionally ignore interest points with a blob response below a threshold.
    double  dfBlobResponseThreshold = 0.0; 
    MissionReader.GetConfigurationParam("BlobResponseFilter", dfBlobResponseThreshold);

    AcceleratedCLCalculator calculator(datasetPath,datasetName,outputPath,dfBlobResponseThreshold);
    calculator.DoCalc();
        
    return 0;
}
