#include "FabMap.h"
#include <string>
#include <boost/python.hpp>

namespace bp = boost::python;

namespace
{
    //Some helper and wrapper functions for translating between Python and C++ data structures
    template<class T>
    bp::list CopySTLVector_to_PythonList(const std::vector<T> &vec)
    {
        bp::list pylist;

        unsigned int nMax = vec.size();
        for(unsigned int i=0;i<nMax;i++)
            pylist.append(vec[i]);

        return pylist;
    }

    //Takes a PythonList, assumed to contain only integers, and returns a STL vector
    std::vector<unsigned int> PythonListInt_to_STLVector(const bp::list &l)
    {
        int nLen = bp::extract<int>(l.attr("__len__")());

        std::vector<unsigned int> vec;
        vec.reserve(nLen);
        for(int i=0;i<nLen;i++)
            vec.push_back((unsigned int)(bp::extract<int>(l[i])));

        return vec;
    }

    bp::list ProcessObservationWithFabMap(FabMapCalculator& fm,const bp::list &WordsFromPython)
    {
        const Observation obs = PythonListInt_to_STLVector(WordsFromPython);    //Python list -> STL vector
        
        vector<double> ComputedLocationProbabilies;                             //Pass to FabMap
        fm.ProcessObservation(obs,ComputedLocationProbabilies);

        return CopySTLVector_to_PythonList(ComputedLocationProbabilies);        //Return to Python
    }

    bp::list ProcessObservationWithFabMap_LikelihoodOnly(FabMapCalculator& fm,const bp::list &WordsFromPython)
    {
        const Observation obs = PythonListInt_to_STLVector(WordsFromPython);        //Python list -> STL vector

        vector<double> ComputedLocationProbabilies;                                 //Pass to FabMap
        fm.ProcessObservation_ComputeLikelihoodOnly(obs,ComputedLocationProbabilies);

        return CopySTLVector_to_PythonList(ComputedLocationProbabilies);            //Return to Python
    }

    unsigned int ProcessObservationWithFabMap_ComputePlaceWithHighestLikelihood(FabMapCalculator& fm,const bp::list &WordsFromPython)
    {
        const Observation obs = PythonListInt_to_STLVector(WordsFromPython);        //Python list -> STL vector
        return fm.ProcessObservation_ComputePlaceWithHighestLikelihood(obs);
    }

    void AddLocation(FabMapCalculator& fm,const bp::list &WordsFromPython)
    {
        const Observation obs = PythonListInt_to_STLVector(WordsFromPython);        //Python list -> STL vector
        fm.AddToObservationToMap(obs);
    }

    unsigned int DetermineVocabSize(string vocab_path, string vocabName)
    {
        unsigned int vocab_size;
        ParseOXV_PeekDimensions(vocab_path + vocabName + ".oxv",vocab_size);
        return vocab_size;
    }

}


//Create thin wrapper classes for overloaded member functions
void    (FabMapCalculator::*ConfirmLastMatch_0)()  = &FabMapCalculator::ConfirmLastMatch;
void    (FabMapCalculator::*ConfirmLastMatch_1)(const int)  = &FabMapCalculator::ConfirmLastMatch;


// Python requires an exported function called init<module-name> in every
// extension module. This is where we build the module contents.
using namespace boost::python;
BOOST_PYTHON_MODULE(fabmap_python)
{   
    def("DetermineVocabSize",&DetermineVocabSize,
        "Read the number of words in the vocabulary from the .oxv vocabulary file"
        "\n Param1: string VocabularyPath"
        "\n Param2: string VocabularyName"
        "\n Returns: int   NumberOfWordsInVocab"
        );

    class_<FabMapCalculator, boost::noncopyable>("FabMapCalculator", init<std::string,std::string,double,double,double,unsigned int,double>
            ("Initialize a FabMap object."
            "\n Param1: string VocabularyPath"
            "\n Param2: string VocabularyName"
            "\n Param3: double p_observe_given_exists"
            "\n Param4: double p_observe_given_not_exists"
            "\n Param5: int vocab_size"
            "\n Param6: double likelihood_smoothing_factor\n"
         ))
        .def("ConfigureForExternalCalls", &FabMapCalculator::ConfigureForExternalCalls,
            "Load settings from a configuration file, and set up relevant structures"
            "\n Param1: string ConfigFilePath"
            "\n Returns: Nothing\n"
            )
        .def("ProcessObservation", &ProcessObservationWithFabMap, 
            "Takes a bag-of-words, and returns a probability distribution over locations in the map"
            ".\n Param1: python_list BagOfWords, a list of the indices of the words present in the current observation"
            "\n Returns: python_list PDF over places in the map. The last entry in the list is special - it corresponds to the new place probability.\n"
            )
        .def("ConfirmLastMatch", ConfirmLastMatch_0, 
            "This function allows the user to externally verify the loop closure suggested by FabMap."
            " If, after calling ProcessObservation, the user does not call ConfirmLastMatch, FabMap will assume data association was rejected, and add the last observation to the map as a new place."
            "(Spuriously adding a new place is much less bad than wrong data association)"
            "\nParameters: None"
            "\nReturns: Nothing\n"
            )
        .def("ConfirmLastMatch", ConfirmLastMatch_1, 
            "This function allows the user to externally override the loop closure suggested by FabMap."
            " So for example you might geometrically verify the top five most likely places suggested by FabMap and choose to associate the observation with say the 3rd most likely place."
            " If that place has ID 27, then call ConfirmLastMatch(27)"
            "\n Param1: ActualMatchingPlaceID"
            "\n Returns: Nothing\n"
            )
        .def("DiscardInfoFromLastMatch", &FabMapCalculator::DiscardInfoFromLastMatch, 
            "There is also the option to discard the last observation, neither creating a new place nor updating an existing place"
            "Normally FabMap never uses this option, however if it makes sense for your system it can be done by calling this function"
            "\nParameters: None"
            "\nReturns: Nothing\n"
            )
        .def("Reset",&FabMapCalculator::FabMap_Restart,
            "Delete all places in the map."
            "\n Parameters: None"
            "\n Returns: Nothing \n"
            )
        .def("LoadAllScenes", &FabMapCalculator::LoadAllScenes,
            "Create places in the map for all scenes specified in the config file."
            "\nUseful for using FabMap as a static image search engine."
            "\n Parameters: None"
            "\n Returns: Nothing \n"            
            )
        .def("ProcessObservation_ComputeLikelihoodOnly", &ProcessObservationWithFabMap_LikelihoodOnly,
            "Takes a bag-of-words, and returns a probability distribution over locations in the map"
            "No prior is applied, and the external user is responsible for adding new places"
            ".\n Param1: python_list BagOfWords, a list of the indices of the words present in the current observation"
            "\n Returns: python_list PDF over places in the map. The last entry in the list is special - it corresponds to the new place probability.\n"
            )
        .def("ComputeMostLikelyMatch", &ProcessObservationWithFabMap_ComputePlaceWithHighestLikelihood,
            "Compute the most likely place in the map to have generated the observation"
            ".\n Param1: python_list BagOfWords, a list of the indices of the words present in the current observation"
            "\n Returns: Index of the most likely place\n"
            )
       .def("AddLocationToMap", &AddLocation,
            "Takes a bag-of-words, and uses it to create a new place in the map"
            "\n Param1: python_list BagOfWords, a list of the indices of the words present in the current observation"
            "\n Returns: Nothing.\n"
            )
       ;
}
