#include <list>
#include <string.h>
#pragma warning(disable: 4996) //warnings for non-secure functions

namespace KMeansIO
{
    //Functor for filtering directory listings
    class NotDesiredType
    {
    public:
        NotDesiredType(string i_sDesiredExt) : sDesiredExt(i_sDesiredExt) {}

        bool operator () (const string& sFilename)
        {
            string sExt= sFilename.substr(sFilename.rfind(".")+1);
            return !(sExt == sDesiredExt);
        }
    private:
        const std::string sDesiredExt;
    };

    template <typename ContainerSupportingPushBack>
    bool ParseFile(const std::string &sFile,ContainerSupportingPushBack &data,unsigned int &nDescriptorDimension,const bool bFilterInputDataOnBlobResponse, const double dfBlobFilterThreshold)
    {
        //The file format is
        //Line 1: Dimensionality
        //Line 2: Number of points
        //(Then, each line is a point)
        //For SIFT/SURF descriptors, the descriptor may be prefixed with some image coordinate parameters (typically 5 doubles).

        FILE* FEATUREFILE = fopen(sFile.c_str(),"rt");
        char sLine[10000];
        char* token;
        char seps[]   = " ,\t\n";

        if(!FEATUREFILE)
        {
            cout << "File " << sFile.c_str() << " could not be found" << endl;
            return false;
        }
        if(ferror(FEATUREFILE))
        {
            cout << "There was an error opening file: " << sFile.c_str() << endl;
            return false;
        }

        //Get dimensionality
        fgets(sLine,sizeof(sLine),FEATUREFILE);
        nDescriptorDimension = (unsigned int) atoi(sLine);     

        //Skip second line (number of points)
        fgets(sLine,sizeof(sLine),FEATUREFILE);

        //Check if there is any prefixed info (e.g. a descriptor patch location), by checking the length of the first point.
        //We will discard this info
        unsigned int nDataLength = 0;
        fpos_t firstPointStart;    fgetpos(FEATUREFILE,&firstPointStart);
        if(!feof(FEATUREFILE) && fgets(sLine,sizeof(sLine),FEATUREFILE)!=NULL)
        {
            token = strtok( sLine, seps );
            while( token != NULL )
            {
                ++nDataLength;
                token = strtok( NULL, seps ); 
            }
        }
        unsigned int PrefixedInfoSize = nDataLength - nDescriptorDimension;
        //Rewind the stream so that it points to the start of the first data point.
        rewind(FEATUREFILE);
        fgets(sLine,sizeof(sLine),FEATUREFILE);
        fgets(sLine,sizeof(sLine),FEATUREFILE);


        //Now, read in the points
        while(!feof(FEATUREFILE))
        {
            if(fgets(sLine,sizeof(sLine),FEATUREFILE)!=NULL)
            {
                if(strlen(sLine)>1)
                {
                    token = strtok( sLine, seps );

                    //Optionally discard points with a blob response less than a threshold
                    bool bMeetsBlobThresh = true;
                    for(unsigned int i=0;i<PrefixedInfoSize;i++)
                    {
                        if(i==0 && bFilterInputDataOnBlobResponse)
                        {
                            double dfBlobResponse;
                            sscanf(token,"%lf",&dfBlobResponse);
                            if(dfBlobResponse < dfBlobFilterThreshold)
                                bMeetsBlobThresh = false;
                        }
                        token = strtok( NULL, seps );   //Discard reast of prefix info
                    }

                    if(bMeetsBlobThresh)
                    {
                        for(unsigned int i=0;i<nDescriptorDimension;i++)
                        {
                            double dfD;
                            sscanf(token,"%lf",&dfD);
                            data.push_back(dfD);

                            //Get next token
                            token = strtok( NULL, seps );
                        }
                    }
                }
            }
        }

        fclose(FEATUREFILE);

        return true;         
    }

    template <typename ContainerSupportingPushBack>
    void read_data_from_directory(const std::string &sDirectory,const std::string &sFileType,ContainerSupportingPushBack &data,unsigned int &dim,const bool bFilterInputDataOnBlobResponse, const double dfBlobFilterThreshold, const bool bQuiet=false)
    {
        //Get a list of images in the directory
        string sFile;
        list<string> sContents;
        if(GetDirectoryContents(sDirectory,sContents))
        {
            sContents.remove_if( NotDesiredType(sFileType) );
            sContents.sort();
            if(!bQuiet)
                cout << "Loading " << sContents.size() << " items from directory...";
            int counter = 0;   
            while(!sContents.empty())
            {           
                sFile = sContents.front();
                sContents.pop_front();
                ParseFile(sDirectory+sFile,data,dim,bFilterInputDataOnBlobResponse,dfBlobFilterThreshold);

                if(++counter % 50 == 0 && !bQuiet)
                {
                    cout << counter << "...";
                }
            }
            if(!bQuiet)
                cout << "Done." << endl;
        }
    }

    //Want to get all dat files in all "FeatureType" sub-sub-directories of base_dir
    //This is particularly for Ingmar's data.
    template <typename ContainerSupportingPushBack>
    void read_data_from_directory_tree(const std::string &sDirectory,const std::string &sSubdir,const std::string &sFileType,ContainerSupportingPushBack &data,unsigned int &dim,const bool bFilterInputDataOnBlobResponse, const double dfBlobFilterThreshold)
    {
        //Get a list of subdirectories in the base directory
        string sImmediateSubdir;
        list<string> sContents;
        bool bGetFiles = false;
        if(GetDirectoryContents(sDirectory,sContents,bGetFiles))    //Get all the directories in base_dir
        {
            sContents.sort();
            cout << "Reading " << sContents.size() << " directories from base directory...";
            int counter = 0;   
            while(!sContents.empty())
            {           
                sImmediateSubdir = sContents.front();
                sContents.pop_front();
                bool bQuiet = true;
                read_data_from_directory(sDirectory + sImmediateSubdir + "/" + sSubdir,sFileType,data,dim,bFilterInputDataOnBlobResponse,dfBlobFilterThreshold,bQuiet);

                if(++counter % 50 == 0)
                {
                    cout << counter << "...";
                }
            }
            cout << "Done." << endl;
        }
    }

    template <typename ContainerSupportingPushBack>
    void read_data_from_file(const std::string &sFile,ContainerSupportingPushBack &data,unsigned int &dim,const bool bFilterInputDataOnBlobResponse, const double dfBlobFilterThreshold)
    {
        ParseFile(sFile,data,dim,bFilterInputDataOnBlobResponse,dfBlobFilterThreshold);   
    }

}//End Namespace KMeanIO
