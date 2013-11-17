#ifndef Parsing_Helper_Functions_XHFNJ_h
#define Parsing_Helper_Functions_XHFNJ_h

#include <string>

#ifndef _WIN32
#include <unistd.h>
#include <sys/times.h>
#include <sys/time.h>
#include <termios.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

#ifdef _WIN32
#include "windows.h"
#include "winbase.h"
#include "winnt.h"
#include <conio.h>
#endif


//Some filename parsing functions
std::string EnsurePathHasTrailingSlash(std::string path);
std::string RemoveTrailingSlash(std::string path);

//Output directory creation
//Given a base directory /foo/
//it creates a subdirectory /foo/n/
//where n is the first available number starting from 1
//and returns the path as a string
//Very useful for storing output from multiple program runs
std::string CreateOutputDirectory(std::string output_path);

#endif //Parsing_Helper_Functions_XHFNJ_h
