#ifndef MAT_parser_h
#define MAT_parser_h

#include <vnl/vnl_matlab_filewrite.h>
#include <vnl/vnl_matlab_read.h>
#include <vcl_new.h>
#include <vector>

//Read in
void ParseMatFile(std::string path, std::string datasetName, std::string varname, std::vector<double> &M);
void ParseMatFile(std::string path, std::string datasetName, std::string varname, std::vector<float> &M);
void ParseMatFile(std::string path, std::string datasetName, std::string varname, std::vector<int> &M);
void ParseMatFile(std::string path, std::string datasetName, std::string varname, vnl_vector<double> &M);
void ParseMatFile(std::string path, std::string datasetName, std::string varname, vnl_vector<int> &M);
void ParseMatFile(std::string path, std::string datasetName, std::string varname, vnl_matrix<double> &M);
//Write out
void WriteToFile(const vnl_matrix<double> &M, std::string varname, std::string datasetName, std::string directory);
void WriteToFile(const vnl_matrix<unsigned int> &M, std::string varname, std::string datasetName, std::string directory);
void WriteToFile(const vnl_vector<double> &M, std::string varname, std::string datasetName, std::string directory);
void WriteToFile(const vnl_vector<int> &M, std::string varname, std::string datasetName, std::string directory);
void WriteToFile(const std::vector<unsigned int> &V, std::string varname, std::string datasetName, std::string directory);
void WriteToFile(const std::vector<double> &V, std::string varname, std::string datasetName, std::string directory);

//Convenience function for output to Matlab - becuase vnl_copy generates linker errors for some reason
void cast_vnl_matrix(const vnl_matrix<unsigned int> &S, vnl_matrix<double> &T);


#endif //MAT_parser_h
