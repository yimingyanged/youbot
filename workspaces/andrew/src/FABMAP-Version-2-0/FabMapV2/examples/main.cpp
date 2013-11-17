#include "FabMap.h"
#include <iomanip>
#include <functional>
#include <vector>

//*****************************************************************************
//
// Example program showing how to use FabMap from within other C++ code.
// See ExternalAPI.cpp in the FabMapV2 directory for a full list of API methods.
//
//*****************************************************************************

// Helper function.
namespace {
	void PrintLocationProbabilityVector(LocationProbabilityContainer ComputedLocationProbability) {
		cout << "Number of locations in the map: "<< ComputedLocationProbability.size() << endl;
		cout << "Probability of each location: " << endl;
        for (unsigned int i = 0; i < ComputedLocationProbability.size();  i++) {
            cout << ComputedLocationProbability[i] << " ";
        }
		cout << endl;
	}
}

// Example program.
int main(int argc, char * argv[])
{
	//*****************************************************************************
	// First we read the FabMap configuration file and configure a FabMap object.
	//*****************************************************************************
	const char * sMissionFile = "Config.moos";
    const char * sMOOSName    = "pFabMapV2";
    
    switch(argc)
    {
        case 3:
            sMOOSName    = argv[2];
        case 2:
            sMissionFile = argv[1];
    }
	        
    string vocab_path, vocabName;
    double p_observe_given_exists, p_observe_given_not_exists, p_at_new_place, df_likelihood_smoothing_factor;
    
    CProcessConfigReader FileReader;
    FileReader.SetFile(sMissionFile);
    
	if (!FileReader.IsOpen()) {
		cout << "Could not find config file: " << sMissionFile << endl
   			 << "Example call: FabMapV2 Config.moos" << endl
   			 << "Sample config files are available in the FabMapV2/config directory" << endl;
		exit(0);
	}

    FileReader.GetValue("VocabPath", vocab_path);
    FileReader.GetValue("VocabName", vocabName);
    FileReader.GetValue("P_OBSERVE_GIVEN_EXISTS", p_observe_given_exists);
    FileReader.GetValue("P_OBSERVE_GIVEN_NOT_EXISTS", p_observe_given_not_exists);
    FileReader.GetValue("LIKELIHOOD_SMOOTHING_FACTOR", df_likelihood_smoothing_factor);
    FileReader.GetValue("P_AT_NEW_PLACE", p_at_new_place);

    unsigned int vocab_size;
    if(ParseOXV_PeekDimensions(vocab_path + vocabName + ".oxv",vocab_size))
    {
		//*****************************************************************************
		// Now, go create a FabMap object.
		//*****************************************************************************
        FabMapCalculator fabmap(vocab_path, vocabName, 
							    p_observe_given_exists, p_observe_given_not_exists,
                                p_at_new_place, vocab_size,df_likelihood_smoothing_factor);
		cout << " Done creating FabMap object!" << endl;

		// After creating a new FabMap object, this function must be called for the object to be usable.
		// Takes as input the "Config.moos" file with FabMap configuration parameters.
        fabmap.ConfigureForExternalCalls(sMissionFile);
		cout << " FabMap configured and ready for use!" << endl;

		// Now the FabMap object is ready to start handling observations.
		// The input should be bag-of-words vectors.
		// You can use the WordMaker library to generate these observations from images
		// (or generate them with your own code. Note that in this case you must make sure FabMap config file points to the word frequency data for your vocabulary).


		// For the purposes of this example we will create some simple dummy observation vectors.
        
		// This observation will contain visual words 1, 14 and 32.
		Observation observation_A(vocab_size,0);
		observation_A[0]=1;
		observation_A[14]=1;
		observation_A[32]=1; 

		// We now supply this observation to FabMap, and get back a PDF over locations in the map.
        LocationProbabilityContainer ComputedLocationProbability;        
        fabmap.ProcessObservation(observation_A, ComputedLocationProbability);
		cout << endl << " Added an observation to FabMap!" << endl;
		// The LocationProbabilityContainer now contains the probability that this observation
		// came from each location in the map. Becuase this is the first observation, a new map is created
		// and the observation is assigned to location zero with probability 1.
		PrintLocationProbabilityVector(ComputedLocationProbability);

		// Supply another observation.
		// This observation will contain visual words 3, 14 and 32.
		Observation observation_B(vocab_size,0);
		observation_B[3]=1;
		observation_B[14]=1;
		observation_B[32]=1;
		fabmap.ProcessObservation(observation_B, ComputedLocationProbability);
		cout << endl << " Added another observation to FabMap!" << endl;
		// Print the pdf.
		PrintLocationProbabilityVector(ComputedLocationProbability);
		// Note that FabMap gives some probably the same place as the first observation, an
		// some probability that it came from a new location not yet in the map.

		// At this point you could do a geometric check between the two images that form the putative loop closure.
		// (There is no geometric check in this code release of FabMap. Adding one will boost
		// accuracy a lot!). Alternatively you could also use position information from a metric
		// SLAM system to rule out unlikely loop closures.

		// If you have done such a check and would like to pass this information back to FabMap, call:
		// fabmap.ConfirmLastMatch();
		// or fabmap.ConfirmLastMatch(placeID); 
		// See fabmap.h and ExternalAPI.cpp for more details.
		
		// If you have no external verification information, DO NOT call ConfirmLastMatch()
		// Not calling ConfirmLastMatch() will result in FabMap creating a new place in the map for every observation.
		// It is much much less bad to incorrectly add a new place to the map
		// than to incorrectly confirm a loop closure. Incorrectly adding a location will lead
		// to a very gradual drop in performance over time, however confirming an incorrect
		// loop closure will break the system almost immediately.
		// See Mark Cummins' thesis for discussion.
    }
    
    return 0;
}



