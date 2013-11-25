#include "WordMaker.h"
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

    bp::list GetBagOfWords(CWordMaker & wm,const std::string ImagePath_andFilename)
    {       
        std::vector<int> ReturnedWords;                             
        wm.GetBagOfWordsForImage(ImagePath_andFilename,ReturnedWords);  //Pass to WordMaker
        return CopySTLVector_to_PythonList(ReturnedWords);              //Return to Python
    }

}


// Python requires an exported function called init<module-name> in every
// extension module. This is where we build the module contents.
using namespace boost::python;
BOOST_PYTHON_MODULE(wordmaker_python)
{   
    class_<CWordMaker, boost::noncopyable>("WordMaker", init<>
            ("Initialize a WordMaker object.")
        )
        .def("LoadVocabularyAndSetup",&CWordMaker::LoadVocabularyAndSetup,
            "Load the vocabulary, and set up relevant structures"
            "\n Param1: string VocabPath"
            "\n Param2: string VocabName"
            "\n Param3: int DescriptorDimensionality"
            "\n Returns: bool success\n"
            )
        .def("GetBagOfWordsForImage",&GetBagOfWords,
            "Takes an image (path), and returns a bag of words"
            "\n Param1: string Path to image. e.g. c:/foo/myimage.jpg"
            "\n Returns: python_list bag-of-words\n"
            )
        .def("SetSURFThreshold",&CWordMaker::SetSURFThreshold,
            "Set the SURF interest point response threshold (default 25)"
            "\n Param1: double SURF_interest_point_threshold"
            "\n Returns: Nothing\n"
            )
        .def("SetSURFOptions", &CWordMaker::SetSURFOptions,
            "Set the SURF descriptor options"
            "Note that these must match the loaded vocabulary"
            "\n Param1: bool SURF_upright Comment: Set true to use USURF, without rotation invariance"            
            "\n Param2: bool SURF_128 Comment: Set true to use SURF-128, the 128 dimensional version of the descriptor"
            "\n Returns: Nothing\n"
            )
        ;
}
