#include "IOHelperFunctions.h"
#include <iostream>
#include <sstream>

std::string EnsurePathHasTrailingSlash(std::string path)
{
    std::string::size_type idx = path.rfind("/");
    if(idx == std::string::npos)
    {
        //Maybe the path was written using \\ instead of /
        idx = path.rfind("\\");
        if(idx == std::string::npos)
        {
            //Found neither '/' or '\\', default to adding '/'
            path += "/";
        }
        else if(idx != path.size()-1)
        {
            path += "\\\\";     //Adds '\\', need 4 because '\' is an escape character
        }
    }
    else if(idx != path.size()-1)
    {
        path += "/";
    }

    return path;
}

std::string RemoveTrailingSlash(std::string path)
{
    std::string::size_type idx = path.rfind("/");
    if(idx == std::string::npos)
    {
        //Maybe the path was written using \\ instead of /
        idx = path.rfind("\\\\");
        if(idx == path.size()-1)
        {
            path = path.substr(0,path.size()-2);
        }
    }
    else if(idx == path.size()-1)
    {
        path = path.substr(0,path.size()-1);
    }

    return path;
}

std::string CreateOutputDirectory(std::string output_path)
{
    //Make a unique subdir in output_path, named with the first available number starting from 1
    //Useful for storing the output of multiple program runs
    int i=1;
    int nMaxTries = 250;
    //If dir exists, we'll go around the loop again, try another number
    //We finish when we create a dir, or hit an error other than "directory exists"
    for(;i<nMaxTries;++i)
    {
        std::stringstream ss;
        ss << i;

        //PMN comments out MAC OSX compilation issue
        //MJC changes it back because it breaks things.
        //MOOSCreateDirectory((output_path + ss.str() + "/"));

    #if _WIN32
        int bOK  = ::CreateDirectory((output_path + ss.str() + "/").c_str(),NULL);
        
        if(!bOK)
        {
            DWORD TheError = GetLastError();        
            if(TheError!=ERROR_ALREADY_EXISTS)    
            {
                std::cout << "Failed to create output directory at path:" << std::endl << output_path << std::endl;
                break;
            }
        }
        else
        {
            //We made a directory, so we're done
            break;
        }
    #else
        if(mkdir((output_path + ss.str() + "/").c_str(),0755)!=0)
        {
            if(errno != EEXIST)
            {
                std::cout << "Failed to create output directory at path:" << std::endl << output_path << std::endl;
                break;
            }
        }
        else
        {
            //We made a directory, so we're done
            break;
        }
    #endif
    }
    std::stringstream ss; ss << i;    
    return output_path + ss.str() + "/";
}
