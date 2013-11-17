#ifndef OXS_parser_h
#define OXS_parser_h

#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <fstream>
#include <stdio.h>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <vnl/vnl_matrix.h>
#include <boost/dynamic_bitset.hpp>
#include <TypeDefinitions.h>
#include "MOOSGenLib/MOOSGenLibGlobalHelper.h"
#include "OXS_parser_helper.h"

//These versions used named return value to avoid copy on return and allow container to be explicitly created and destroyed.
bool ParseOXS(const std::string &sScenesFilename, boost::numeric::ublas::compressed_matrix<unsigned int> &m); //Sparse
bool ParseOXS(const std::string &sScenesFilename, std::vector<boost::numeric::ublas::compressed_vector<unsigned int> > &m); //Sparse
bool ParseOXS(const std::string &sScenesFilename, std::vector<boost::numeric::ublas::compressed_vector<unsigned int> > &scenes, std::map<std::string,unsigned int> &FilenameToSceneID);  //Sparse, with mapping from filename to scene ID.
bool ParseOXS(const std::string &sScenesFilename, SceneRecordConatiner &SceneRecords, const bool bLoadGeometry, const double dfBlobThreshold);
bool ParseOXS(const std::string &sScenesFilename, SceneRecordConatiner &SceneRecords, std::map<std::string,unsigned int> &FilenameToSceneID);  //Sparse, with mapping from filename to scene ID.
bool ParseOXS(const std::string &sScenesFilename, vnl_matrix<unsigned int> &scenes);                              //Dense
vnl_matrix<unsigned int> ParseOXStoVNLMatrix(const std::string &sScenesFilename);                              //Convenience method    
bool ParseOXStoInvertedIndex(const std::string &sScenesFilename, std::vector<boost::dynamic_bitset<> > &WordToScenesIndex);
bool ParseOXStoInvertedIndex(const std::string &sScenesFilename, std::vector<std::vector<unsigned int> > &WordToScenesIndex,const double dfBlobThreshold = 0.0);
bool ParseOXV_PeekDimensions(const std::string &sVocabFilename, unsigned int &vocab_size);

bool ParseOXS_PeekDimensions_SingleFile(const std::string sScenesDir,const std::string sScenesFile, unsigned int &num_scenes, unsigned int &vocab_size);
bool ParseOXS_PeekDimensions(std::string sScenesDir,std::string sScenesFile, unsigned int &num_scenes, unsigned int &vocab_size);
bool ParseOXS_PeekDimensions(std::string sScenesDir,std::string sScenesFile, unsigned int &num_scenes);

//Meta-OXS capable functions
bool ParseOXS(std::string sScenesDir,std::string sScenesFile, SceneRecordConatiner &SceneRecords, const bool bLoadGeometry, const double dfBlobThreshold);
bool ParseOXS(string sScenesDir,string sScenesFile,string sExcludedRegionsFilePath,SceneRecordConatiner &SceneRecords, const bool bLoadGeometry, const double dfBlobThreshold);
bool ParseOXS(string sScenesDir,string sScenesFile, SceneRecordConatiner &SceneRecords, const InterestPointIsInExcludedRegionCheck &IPExclusionCheck, const bool bLoadGeometry, const double dfBlobThreshold);
bool ParseOXS_SingleFile(const string sScenesDir,const string sScenesFile, SceneRecordConatiner &SceneRecords, const InterestPointIsInExcludedRegionCheck &IPExclusionCheck, const bool bLoadGeometry, const double dfBlobThreshold);

bool IsOXSNewFormat(const string sScenesDir,const string sScenesFile);              //Check if there's geometry present in the file.
bool IsOXSNewFormat_SingleFile(const string sScenesDir,const string sScenesFile);

#endif
