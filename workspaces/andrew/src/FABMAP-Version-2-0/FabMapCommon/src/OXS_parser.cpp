#include "OXS_parser.h"

using namespace boost::numeric::ublas;


using namespace std;

//Lots of very similar parsing functions for slightly different circumstances or container types
//Really need to clean these up, eliminate as much code as possible with proper template use.
vnl_matrix<unsigned int> ParseOXStoVNLMatrix(const string &sScenesFilename)
{
    unsigned int num_scenes, vocab_size;
	string sPath, sName, sExt;
	MOOSFileParts(sScenesFilename,sPath,sName,sExt);
    ParseOXS_PeekDimensions(sPath,sName+sExt, num_scenes, vocab_size);

    vnl_matrix<unsigned int> scenes(num_scenes,vocab_size);
    ParseOXS(sScenesFilename,scenes);
    return scenes;
}

bool ParseOXStoInvertedIndex(const string &sScenesFilename, std::vector<boost::dynamic_bitset<> > &WordToScenesIndex)
{
    MOOSTrace("Creating inverted index from scenes file...");
    double dfStart = HPMOOSTime();//performance timer

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;
    std::string sWhichWords;
    std::string sHowOften;

    int curr_scene =0;
    while(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');

        //Look for the keyword we want
        size_t nPos = sLine.find("SCENE:");
        if(nPos!=std::string::npos)
        {
            //Skip 2 lines, then grab the data
            getline(ScenesFile,sLine,'\n');getline(ScenesFile,sLine,'\n');
            //Read what words occured in this scene
            getline(ScenesFile,sWhichWords,'\n');
            //And how often
            getline(ScenesFile,sHowOften,'\n');

            //Defend against infuriating Linux/Windows line ending differences
            MOOSRemoveChars(sWhichWords,"\r");
            MOOSRemoveChars(sHowOften,"\r");

            while(sWhichWords.size() != 0)
            {
                //And then parse the the result and put into some convenient structure
                int word_id = atoi((MOOSChomp(sWhichWords," ")).c_str());
                unsigned int num_instances = atoi((MOOSChomp(sHowOften," ")).c_str());

                //Commit to some convenient structure
                WordToScenesIndex[word_id][curr_scene] = 1;
            }
            //Increment our scene counter
            ++curr_scene;
        }
    }

    cout << "Done in " << HPMOOSTime()-dfStart << " seconds." << endl;
    ScenesFile.close();
    return true;
}

bool ParseOXStoInvertedIndex(const string &sScenesFilename, std::vector<std::vector<unsigned int> > &WordToScenesIndex, const double dfBlobThreshold)
{
    MOOSTrace("Creating inverted index from scenes file...");
    double dfStart = HPMOOSTime();//performance timer

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;

    //Check the format - is this an old OXS file with wordIDs only, or a new one with blob and geometric info too
    bool bNewOXSFormat = false;
    bool bFormatDetermined = false;
    unsigned int sceneRecLen = 0;
    while(!bFormatDetermined && !ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');
        //Look for the start of a scene record
        size_t nPos = sLine.find("SCENE:");
        if(nPos!=std::string::npos && sceneRecLen==0)
        {
            //Found the start of the first scene record. Start counting.
            sceneRecLen++;       
        }
        else if(nPos!=std::string::npos && sceneRecLen!=0)
        {
            //Found the end of the first scene record
            bFormatDetermined = true;
            if(sceneRecLen == 10)
            {
                bNewOXSFormat = true;
            }
        }
        else if(sceneRecLen!=0)
        {
            //We've passed the start of the first scene rec, but not reached the end. Count.
            sceneRecLen++;
        }
    }
    ScenesFile.seekg(0, ios::beg);  //Rewind to start of file

    bool bFilterBlobResponse = bNewOXSFormat && dfBlobThreshold > 0.0;

    if(bNewOXSFormat && bFilterBlobResponse)
        MOOSTrace("Filtering blob response at %3.1f...",dfBlobThreshold);
    else if(!bNewOXSFormat && bFilterBlobResponse)
        MOOSTrace("No blob response info in file, no filtering applied...",dfBlobThreshold);

    std::string sWhichWords;
    std::string sHowOften;
    std::string sBlobResponse;

    int curr_scene =0;
    unsigned int nMaxUnique = 0;
    while(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');

        //Look for the keyword we want
        size_t nPos = sLine.find("SCENE:");
        if(nPos!=std::string::npos)
        {
            //Skip 2 lines, then grab the data
            getline(ScenesFile,sLine,'\n');getline(ScenesFile,sLine,'\n');
            //Read what words occured in this scene
            getline(ScenesFile,sWhichWords,'\n');
            //And how often
            getline(ScenesFile,sHowOften,'\n');

            //Defend against infuriating Linux/Windows line ending differences
            MOOSRemoveChars(sWhichWords,"\r");
            MOOSRemoveChars(sHowOften,"\r");

            //If the OXS file has blob response info, and we want it, go read that too
            if(bFilterBlobResponse && bNewOXSFormat)
            {
                //Ignore all fields except blob response
                getline(ScenesFile,sLine,'\n');
                getline(ScenesFile,sBlobResponse,'\n');
                getline(ScenesFile,sLine,'\n');
                getline(ScenesFile,sLine,'\n');
                getline(ScenesFile,sLine,'\n');
                MOOSRemoveChars(sBlobResponse,"\r");
            }

            unsigned int nWordsRead = 0;
            while(sWhichWords.size() != 0)
            {
                //And then parse the the result and put into some convenient structure
                int word_id = atoi((MOOSChomp(sWhichWords," ")).c_str());
                unsigned int num_instances = atoi((MOOSChomp(sHowOften," ")).c_str());

                //If we're filtering on blob response, go read that as well
                double dfMaxBlob = 0.0;
                if(bFilterBlobResponse && bNewOXSFormat)
                {
                    for(unsigned int g =0;g<num_instances;++g)
                    {
                        double dfBlob = atof((MOOSChomp(sBlobResponse," ")).c_str());
                        dfMaxBlob = dfBlob > dfMaxBlob ? dfBlob : dfMaxBlob;
                    }
                }

                //Commit to some convenient structure
                if(!bFilterBlobResponse || (bFilterBlobResponse && dfMaxBlob>=dfBlobThreshold))
                {
                    WordToScenesIndex[word_id].push_back(curr_scene);
                    ++nWordsRead;
                }
            }
            //Increment our scene counter
            ++curr_scene;
            //Record the maximum number of unique words, purely for debugging information
            nMaxUnique = max(nMaxUnique,nWordsRead);
        }
    }

    cout << "Done in " << HPMOOSTime()-dfStart << " seconds." << endl;
    MOOSTrace("Maximum number of unique words in a scene: %d \n",nMaxUnique);
    ScenesFile.close();
    return true;
}


bool ParseOXS(const string &sScenesFilename, boost::numeric::ublas::compressed_matrix<unsigned int> &scenes)
{
    cout << "Reading scenes info from file...";
    double dfStart = HPMOOSTime();//performance timer

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;
    std::string sWhichWords;
    std::string sHowOften;

    int curr_scene =0;
    while(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');

        //Look for the keyword we want
        size_t nPos = sLine.find("SCENE:");
        if(nPos!=std::string::npos)
        {
            //Skip 2 lines, then grab the data
            getline(ScenesFile,sLine,'\n');getline(ScenesFile,sLine,'\n');
            //Read what words occured in this scene
            getline(ScenesFile,sWhichWords,'\n');
            //And how often
            getline(ScenesFile,sHowOften,'\n');

            //Defend against infuriating Linux/Windows line ending differences
            MOOSRemoveChars(sWhichWords,"\r");
            MOOSRemoveChars(sHowOften,"\r");

            while(sWhichWords.size() != 0)
            {
                //And then parse the the result and put into some convenient structure
                int word_id = atoi((MOOSChomp(sWhichWords," ")).c_str());
                unsigned int num_instances = atoi((MOOSChomp(sHowOften," ")).c_str());

                //Commit to some convenient structure
                scenes(curr_scene,word_id) = num_instances;
                //MOOSTrace("The result was %d %d %d\n",curr_scene,word_id,num_instances);
            }
            //Increment our scene counter
            ++curr_scene;
        }
    }

    cout << "Done in " << HPMOOSTime()-dfStart << " seconds." << endl;
    ScenesFile.close();
    return true;
}

bool ParseOXS(const string &sScenesFilename, std::vector<boost::numeric::ublas::compressed_vector<unsigned int> > &scenes)
{
    cout << "Reading scenes info from file...";
    double dfStart = HPMOOSTime();//performance timer

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;
    std::string sWhichWords;
    std::string sHowOften;

    int curr_scene =0;
    while(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');

        //Look for the keyword we want
        size_t nPos = sLine.find("SCENE:");
        if(nPos!=std::string::npos)
        {
            //Skip 2 lines, then grab the data
            getline(ScenesFile,sLine,'\n');getline(ScenesFile,sLine,'\n');
            //Read what words occured in this scene
            getline(ScenesFile,sWhichWords,'\n');
            //And how often
            getline(ScenesFile,sHowOften,'\n');

            //Defend against infuriating Linux/Windows line ending differences
            MOOSRemoveChars(sWhichWords,"\r");
            MOOSRemoveChars(sHowOften,"\r");

            while(sWhichWords.size() != 0)
            {
                //And then parse the the result and put into some convenient structure
                int word_id = atoi((MOOSChomp(sWhichWords," ")).c_str());
                unsigned int num_instances = atoi((MOOSChomp(sHowOften," ")).c_str());

                //Commit to some convenient structure
                scenes[curr_scene](word_id) = num_instances;
                //MOOSTrace("The result was %d %d %d\n",curr_scene,word_id,num_instances);
            }
            //Increment our scene counter
            ++curr_scene;
        }
    }

    cout << "Done in " << HPMOOSTime()-dfStart << " seconds." << endl;
    ScenesFile.close();
    return true;
}

bool ParseOXS(const string &sScenesFilename, std::vector<boost::numeric::ublas::compressed_vector<unsigned int> > &scenes, std::map<std::string,unsigned int> &FilenameToSceneID)
{
    cout << "Reading scenes info from file...";
    double dfStart = HPMOOSTime();//performance timer

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;
    std::string sWhichWords;
    std::string sHowOften;

    unsigned int curr_scene =0;
    while(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');

        //Look for the keyword we want
        size_t nPos = sLine.find("SCENE:");
        if(nPos!=std::string::npos)
        {
            //Read filename
            getline(ScenesFile,sLine,'\n');
            MOOSRemoveChars(sLine,"\r");    //Defend against line ending differences
            string sPath,sFile,sExtension;
            MOOSFileParts(sLine,sPath,sFile,sExtension);
            FilenameToSceneID.insert(make_pair(sFile+"."+sExtension,curr_scene));
            //Skip next line
            getline(ScenesFile,sLine,'\n');
            //Read what words occured in this scene
            getline(ScenesFile,sWhichWords,'\n');
            //And how often
            getline(ScenesFile,sHowOften,'\n');

            //Defend against infuriating Linux/Windows line ending differences
            MOOSRemoveChars(sWhichWords,"\r");
            MOOSRemoveChars(sHowOften,"\r");

            while(sWhichWords.size() != 0)
            {
                //And then parse the the result and put into some convenient structure
                int word_id = atoi((MOOSChomp(sWhichWords," ")).c_str());
                unsigned int num_instances = atoi((MOOSChomp(sHowOften," ")).c_str());

                //Commit to some convenient structure
                scenes[curr_scene](word_id) = num_instances;
                //MOOSTrace("The result was %d %d %d\n",curr_scene,word_id,num_instances);
            }
            //Increment our scene counter
            ++curr_scene;
        }
    }

    cout << "Done in " << HPMOOSTime()-dfStart << " seconds." << endl;
    ScenesFile.close();
    return true;
}

bool ParseOXS_SingleFile(const string sScenesDir,const string sScenesFile, SceneRecordConatiner &SceneRecords, const InterestPointIsInExcludedRegionCheck &IPExclusionCheck, const bool bLoadGeometry, const double dfBlobThreshold)
{
	string sScenesFilename = sScenesDir+sScenesFile;
    double dfStart = HPMOOSTime();//performance timer

    //Ladybug x-coords are relative to the side of the image. Use this value to convert to absolute composite image coords.
    unsigned int nLadybugImageWidth = 384;

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //Check the format - is this an old OXS file with wordIDs only, or a new one with geometric info too
    bool bNewOXSFormat = IsOXSNewFormat_SingleFile(sScenesDir,sScenesFile);

    bool bFilterBlobResponse = bNewOXSFormat && dfBlobThreshold > 0.0;

    bool bFiltersApplied = bFilterBlobResponse || !IPExclusionCheck.empty();

    if(bNewOXSFormat && bLoadGeometry)
        cout << "Reading scenes info and geometry from file...";
    else
        cout << "Reading scenes info from file...";


    ReadRecord SceneRecordReader(ScenesFile,bLoadGeometry,bNewOXSFormat,bFiltersApplied,bFilterBlobResponse,dfBlobThreshold,IPExclusionCheck,nLadybugImageWidth);

     SceneRecord CurrentScene;
     while(SceneRecordReader(CurrentScene))
     {
         SceneRecords.push_back(CurrentScene);
     }
     
    cout << "Done in " << HPMOOSTime()-dfStart << " seconds." << endl;
    ScenesFile.close();
    return true;
}

bool ParseOXS(string sScenesDir,string sScenesFile,SceneRecordConatiner &SceneRecords, const bool bLoadGeometry, const double dfBlobThreshold)
{
    //Make a null excluded region.
    InterestPointIsInExcludedRegionCheck IPExclusionCheck;
    return ParseOXS(sScenesDir,sScenesFile,SceneRecords,IPExclusionCheck,bLoadGeometry,dfBlobThreshold);
}


bool ParseOXS(string sScenesDir,string sScenesFile,string sExcludedRegionsFilePath,SceneRecordConatiner &SceneRecords, const bool bLoadGeometry, const double dfBlobThreshold)
{
    InterestPointIsInExcludedRegionCheck IPExclusionCheck(sExcludedRegionsFilePath);
    return ParseOXS(sScenesDir,sScenesFile,SceneRecords,IPExclusionCheck,bLoadGeometry,dfBlobThreshold);
}

bool ParseOXS(string sScenesDir,string sScenesFile, SceneRecordConatiner &SceneRecords, const InterestPointIsInExcludedRegionCheck &IPExclusionCheck, const bool bLoadGeometry, const double dfBlobThreshold)
{
    //Remove any directory part from sScenesFile and append to sScenesDir
    string sDirPart,sFilePart,sExt;
    MOOSFileParts(sScenesFile,sDirPart,sFilePart,sExt);
    if(!sDirPart.empty())
        sScenesDir += sDirPart + "/";
    sScenesFile = sFilePart + "." + sExt;

	const string sScenesFilename = sScenesDir+sScenesFile;

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;
    if(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');
        size_t nPos = sLine.find("_META_OXS_FILE_");
        if(nPos!=std::string::npos)
        {
			cout << "Reading MetaOXS file..." << endl;
			//It's a meta-OXS file, which specifies several other OXS files to load
			while(!ScenesFile.eof())
			{
				getline(ScenesFile,sLine,'\n');
				size_t nPos = sLine.find("File=");					//Look for the keyword we want
				if(nPos!=std::string::npos)
				{
					std::string sFilename;
		            MOOSRemoveChars(sLine,"\r");				//Defend against infuriating Linux/Windows line ending differences
					MOOSValFromString(sFilename,sLine,"File",true);
					//Recursively read from subfiles
					ParseOXS(sScenesDir,sFilename,SceneRecords,IPExclusionCheck,bLoadGeometry,dfBlobThreshold);
				}
			}
		}
		else
		{
			//It's just a standard OXS file. Read it in.
			ParseOXS_SingleFile(sScenesDir,sScenesFile,SceneRecords,IPExclusionCheck,bLoadGeometry,dfBlobThreshold);
		}
	}

	return true;
}

bool ParseOXS(const string &sScenesFilename, SceneRecordConatiner &SceneRecords, map<string,unsigned int> &FilenameToSceneID)
{
    cout << "Reading scenes info from file...";
    double dfStart = HPMOOSTime();//performance timer

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //Clear container
    SceneRecords.clear();

    //loop variables
    std::string sLine;
    std::string sWhichWords;
    std::string sHowOften;

    unsigned int curr_scene =0;
    while(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');

        //Look for the keyword we want
        size_t nPos = sLine.find("SCENE:");
        if(nPos!=std::string::npos)
        {
            //Read filename
            getline(ScenesFile,sLine,'\n');
            MOOSRemoveChars(sLine,"\r");    //Defend against line ending differences
            string sPath,sFile,sExtension;
            MOOSFileParts(sLine,sPath,sFile,sExtension);
            FilenameToSceneID.insert(make_pair(sFile+"."+sExtension,curr_scene));
            //Skip next line
            getline(ScenesFile,sLine,'\n');
            //Read what words occured in this scene
            getline(ScenesFile,sWhichWords,'\n');
            //And how often
            getline(ScenesFile,sHowOften,'\n');

            //Defend against infuriating Linux/Windows line ending differences
            MOOSRemoveChars(sWhichWords,"\r");
            MOOSRemoveChars(sHowOften,"\r");

            unsigned int nTotalInterestPointsInScene = 0;
            SceneRecord CurrentScene;
            while(sWhichWords.size() != 0)
            {
                //And then parse the the result and put into some convenient structure
                int word_id = atoi((MOOSChomp(sWhichWords," ")).c_str());
                unsigned int num_instances = atoi((MOOSChomp(sHowOften," ")).c_str());

                CurrentScene.WordData.push_back(WordWithInterestPoints((unsigned int)word_id,num_instances));
                nTotalInterestPointsInScene += num_instances;
            }
            CurrentScene.nTotalNumInterestPointsInScene = nTotalInterestPointsInScene;
            CurrentScene.nNumWords = CurrentScene.WordData.size();
            std::sort(CurrentScene.WordData.begin(),CurrentScene.WordData.end());   //Paranoia. Ensure words are sorted.
            SceneRecords.push_back(CurrentScene);
            //Increment our scene counter
            ++curr_scene;
        }
    }

    cout << "Done in " << HPMOOSTime()-dfStart << " seconds." << endl;
    ScenesFile.close();
    return true;
}


bool ParseOXS(const string &sScenesFilename, vnl_matrix<unsigned int> &scenes)
{   //an exact copy of the above function for a different container type.

    cout << "Reading scenes info from file...";
    double dfStart = HPMOOSTime();//performance timer

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;
    std::string sWhichWords;
    std::string sHowOften;

    int curr_scene =0;
    while(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');

        //Look for the keyword we want
        size_t nPos = sLine.find("SCENE:");
        if(nPos!=std::string::npos)
        {
            //Skip 2 lines, then grab the data
            getline(ScenesFile,sLine,'\n');getline(ScenesFile,sLine,'\n');
            //Read what words occured in this scene
            getline(ScenesFile,sWhichWords,'\n');
            //And how often
            getline(ScenesFile,sHowOften,'\n');

            //Defend against infuriating Linux/Windows line ending differences
            MOOSRemoveChars(sWhichWords,"\r");
            MOOSRemoveChars(sHowOften,"\r");

            while(sWhichWords.size() != 0)
            {
                //And then parse the the result and put into some convenient structure
                int word_id = atoi((MOOSChomp(sWhichWords," ")).c_str());
                unsigned int num_instances = atoi((MOOSChomp(sHowOften," ")).c_str());

                //Commit to some convenient structure
                scenes(curr_scene,word_id) = num_instances;
                //MOOSTrace("The result was %d %d %d\n",curr_scene,word_id,num_instances);
            }
            //Increment our scene counter
            ++curr_scene;
        }
    }

    cout << "Done in " << HPMOOSTime()-dfStart << " seconds." << endl;
    ScenesFile.close();
    return true;
}

bool ParseOXS_PeekDimensions(string sScenesDir,string sScenesFile, unsigned int &num_scenes)
{
	unsigned int dummy_vocab_size;
    ParseOXS_PeekDimensions(sScenesDir,sScenesFile, num_scenes, dummy_vocab_size);
    return true;
}

bool ParseOXS_PeekDimensions(string sScenesDir,string sScenesFile, unsigned int &num_scenes, unsigned int &vocab_size)
{
    //Remove any directory part from sScenesFile and append to sScenesDir
    string sDirPart,sFilePart,sExt;
    MOOSFileParts(sScenesFile,sDirPart,sFilePart,sExt);
    if(!sDirPart.empty())
        sScenesDir += sDirPart + "/";
    sScenesFile = sFilePart + "." + sExt;

	num_scenes = 0;
    const string sScenesFilename = sScenesDir+sScenesFile;

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;
    if(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');
        size_t nPos = sLine.find("_META_OXS_FILE_");
        if(nPos!=std::string::npos)
        {
			//It's a meta-OXS file, which specifies several other OXS files to load
            unsigned int nCount = 0;
            unsigned int vocab_size_last_iteration;
			while(!ScenesFile.eof())
			{
				getline(ScenesFile,sLine,'\n');
				size_t nPos = sLine.find("File=");					//Look for the keyword we want
				if(nPos!=std::string::npos)
				{
					std::string sFilename;
		            MOOSRemoveChars(sLine,"\r");				//Defend against infuriating Linux/Windows line ending differences
					MOOSValFromString(sFilename,sLine,"File",true);
					//Recursively read from subfiles
					unsigned int num_scenes_this_file;
					ParseOXS_PeekDimensions(sScenesDir,sFilename, num_scenes_this_file, vocab_size);
					num_scenes += num_scenes_this_file;
                    //Check the vocabulary sizes are consistent
                    if(nCount++ != 0 && vocab_size != vocab_size_last_iteration)
                    {
                        cerr << endl << "ERROR!. Trying to load from a MetaOXS file with inconsistent vocabulary sizes!" << endl << "One has size " << vocab_size << " other has size " << vocab_size_last_iteration << endl << "Aborting" << endl;
                        exit(0);    //This is such a terrible thing that we should quit immediately.
                    }
                    vocab_size_last_iteration = vocab_size;
				}
			}
		}
		else
		{
			//It's just a standard OXS file. Read it in.
			unsigned int num_scenes_this_file;
			ParseOXS_PeekDimensions_SingleFile(sScenesDir,sScenesFile,num_scenes_this_file,vocab_size);
			num_scenes += num_scenes_this_file;
		}
	}
	return true;
}

bool ParseOXS_PeekDimensions_SingleFile(const string sScenesDir,const string sScenesFile, unsigned int &num_scenes, unsigned int &vocab_size)
{
	string sScenesFilename = sScenesDir+sScenesFile;
    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sScenesFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;

    //here we go...
    bool numScenesUnknown = true;
    bool vocabSizeUnknown = true;
    while(!ScenesFile.eof() & (numScenesUnknown || vocabSizeUnknown) )
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');

        //Look for the keyword we want
        size_t nPos = sLine.find("WORDS:");
        if(nPos!=std::string::npos)
        {
            //EXTRACT NUMBER OF WORDS HERE
            int pos = sLine.find(":");
            vocab_size = atoi((sLine.substr(pos+1,sLine.length()-1)).c_str());
            vocabSizeUnknown = false;
        }

        nPos = sLine.find("SCENES:");
        if(nPos!=std::string::npos)
        {
            //EXTRACT NUMBER OF SCENES HERE
            int pos = sLine.find(":");
            num_scenes = atoi((sLine.substr(pos+1,sLine.length()-1)).c_str());
            numScenesUnknown = false;
        }
    }
    ScenesFile.close();
    return true;
}

bool ParseOXV_PeekDimensions(const std::string &sVocabFilename, unsigned int &vocab_size)
{
    std::ifstream VocabFile(sVocabFilename.c_str());
    if(!VocabFile.is_open())
    {
        MOOSTrace("\nERROR: Cannot open:\n %s\n",sVocabFilename.c_str());
        return false;
    }

    //loop variables
    std::string sLine;

    //here we go...
    bool vocabSizeUnknown = true;
    while(!VocabFile.eof() & vocabSizeUnknown)
    {
        //grab a line
        getline(VocabFile,sLine,'\n');

        //Look for the keyword we want
        size_t nPos = sLine.find("WORDS:");
        if(nPos!=std::string::npos)
        {
            //EXTRACT NUMBER OF WORDS HERE
            int pos = sLine.find(":");
            vocab_size = atoi((sLine.substr(pos+1,sLine.length()-1)).c_str());
            vocabSizeUnknown = false;
        }
    }
    VocabFile.close();
    return true;
}


bool IsOXSNewFormat(string sScenesDir,string sScenesFile)
{
    bool bNewOXSFormat = true;

    //Remove any directory part from sScenesFile and append to sScenesDir
    string sDirPart,sFilePart,sExt;
    MOOSFileParts(sScenesFile,sDirPart,sFilePart,sExt);
    if(!sDirPart.empty())
        sScenesDir += sDirPart + "/";
    sScenesFile = sFilePart + "." + sExt;

	const string sScenesFilename = sScenesDir+sScenesFile;

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        cerr << "ERROR: Cannot open:" << endl << sScenesFilename << endl;
        return false;
    }

    //loop variables
    std::string sLine;
    if(!ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');
        size_t nPos = sLine.find("_META_OXS_FILE_");
        if(nPos!=std::string::npos)
        {
			//It's a meta-OXS file, which specifies several other OXS files to load
			while(!ScenesFile.eof())
			{
				getline(ScenesFile,sLine,'\n');
				size_t nPos = sLine.find("File=");					//Look for the keyword we want
				if(nPos!=std::string::npos)
				{
					std::string sFilename;
		            MOOSRemoveChars(sLine,"\r");				//Defend against infuriating Linux/Windows line ending differences
					MOOSValFromString(sFilename,sLine,"File",true);
					//Recursively read from subfiles
					bNewOXSFormat &= IsOXSNewFormat(sScenesDir,sFilename);
				}
			}
		}
		else
		{
			//It's just a standard OXS file. Read it in.
			bNewOXSFormat &= IsOXSNewFormat_SingleFile(sScenesDir,sScenesFile);
		}
	}

	return bNewOXSFormat;
}

bool IsOXSNewFormat_SingleFile(const string sScenesDir,const string sScenesFile)
{
    string sScenesFilename = sScenesDir+sScenesFile;

    std::ifstream ScenesFile(sScenesFilename.c_str());
    if(!ScenesFile.is_open())
    {
        cerr << "ERROR: Cannot open: "  << endl << sScenesFilename << endl;
        return false;
    }

    //loop variables
    std::string sLine;

    //Check the format - is this an old OXS file with wordIDs only, or a new one with geometric info too
    bool bNewOXSFormat = false;
    bool bFormatDetermined = false;
    unsigned int sceneRecLen = 0;
    while(!bFormatDetermined && !ScenesFile.eof())
    {
        //grab a line
        getline(ScenesFile,sLine,'\n');
        //Look for the start of a scene record
        size_t nPos = sLine.find("SCENE:");
        if(nPos!=std::string::npos && sceneRecLen==0)
        {
            //Found the start of the first scene record. Start counting.
            sceneRecLen++;       
        }
        else if(nPos!=std::string::npos && sceneRecLen!=0)
        {
            //Found the end of the first scene record
            bFormatDetermined = true;
            if(sceneRecLen == 10)
            {
                bNewOXSFormat = true;
            }
        }
        else if(sceneRecLen!=0)
        {
            //We've passed the start of the first scene rec, but not reached the end. Count.
            sceneRecLen++;
        }
    }

    return bNewOXSFormat;
}
