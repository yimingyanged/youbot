#include "MAT_parser.h"
using namespace std;
//=====================================
//            Reading in
//=====================================

void ParseMatFile(string path, string datasetName, string varname, std::vector<double> &Vec)
{
    vnl_vector<double> M;
    ParseMatFile(path,datasetName,varname,M);
    Vec.assign(M.begin(),M.end());
}

void ParseMatFile(string path, string datasetName, string varname, std::vector<float> &Vec)
{
    vnl_vector<double> M;
    ParseMatFile(path,datasetName,varname,M);
    Vec.assign(M.begin(),M.end());
}

void ParseMatFile(string path, string datasetName, string varname, std::vector<int> &Vec)
{
    vnl_vector<int> M;
    ParseMatFile(path,datasetName,varname,M);
    Vec.assign(M.begin(),M.end());
}

void ParseMatFile(string path, string datasetName, string varname, vnl_vector<double> &M)
{
    string filename = datasetName + "_" + varname + ".mat";
    vcl_ifstream matFile((path + filename).c_str(),std::ios::binary | std::ios::in);
        
    if(matFile.is_open())
    {
        // MAJOR CAVEAT
        // vnl_matlab_read_or_die does some work inside an assert() statement
        // In Visual Studio (and other compilers too?), assert statements are NOT COMPILED into release builds.
        // Therefore, the program WILL FAIL in an optimized build, however, it works fine in a Debug build.
        // Latest version of VNL source has fixed this bug, but versions of VNL older than 1.09 (I think) will fail in strange ways.
        cout << "Reading in " << varname << " file from disk...";
        bool result = vnl_matlab_read_or_die(matFile, M, varname.c_str());
        if(!result)
        {
            cout << "Error parsing file: " << endl << varname << endl << "from path:" << endl << path << endl;
        }
        else
        {
            cout << "Done" << endl;
            //Sanity check stats on input
            cout << "======Vector Stats========" << endl;
            cout << "Vector size is "  << M.size() << endl;
            cout << "Vector mean is " << M.mean() << endl;
            cout << "Max entry is " << M.max_value() << endl;
            cout << "===============================" << endl << endl;
        }
    }
    else
    {
        cout << "Cannot open file: " << endl << filename << endl << "from path:" << endl << path << endl;
    }

}

void ParseMatFile(string path, string datasetName, string varname, vnl_vector<int> &M)
{
    string filename = datasetName + "_" + varname + ".mat";
    vcl_ifstream matFile((path + filename).c_str(),std::ios::binary | std::ios::in);

    //For Matlab format, values are doubles.
    //So, we'll cast the input vector to ints.
    vnl_vector<double> Md(M.size());
        
    if(matFile.is_open())
    {
        // MAJOR CAVEAT
        // vnl_matlab_read_or_die does some work inside an assert() statement
        // In Visual Studio (and other compilers too?), assert statements are NOT COMPILED into release builds.
        // Therefore, the program WILL FAIL in an optimized build, however, it works fine in a Debug build.
        // Latest version of VNL source has fixed this bug, but versions older than 1.09 (I think) will fail in strange ways.
        cout << "Reading in " << varname << " file from disk...";
        bool result = vnl_matlab_read_or_die(matFile, Md, varname.c_str());
        if(!result)
        {
            cout << "Error parsing file: " << endl << varname << endl << "from path:" << endl << path << endl;
        }
        else
        {
            vcl_destroy(&M);
            new (&M) vnl_vector<int>(Md.size());
            //Cast to int
            for(unsigned int i=0;i<M.size();++i)
                M[i] = (int) Md[i];

            cout << "Done" << endl;
            //Sanity check stats on input
            cout << "======Vector Stats========" << endl;
            cout << "Vector size is "  << M.size() << endl;
            cout << "Vector mean is " << M.mean() << endl;
            cout << "Max entry is " << M.max_value() << endl;
            cout << "===============================" << endl << endl;
        }
    }
    else
    {
        cout << "Cannot open file: " << endl << filename << endl << "from path:" << endl << path << endl;
    }

}

void ParseMatFile(string path, string datasetName, string varname, vnl_matrix<double> &M)
{
    string filename = datasetName + "_" + varname + ".mat";
    vcl_ifstream matFile((path + filename).c_str(),std::ios::binary | std::ios::in);
        
    if(matFile.is_open())
    {
        // MAJOR CAVEAT
        // vnl_matlab_read_or_die does some work inside an assert() statement
        // In Visual Studio (and other compilers too?), assert statements are NOT COMPILED into release builds.
        // Therefore, the program WILL FAIL in an optimized build, however, it works fine in a Debug build.
        // Latest version of VNL source has fixed this bug, but versions older than 1.09 (I think) will fail in strange ways.
        cout << "Reading in " << varname << " file from disk...";
        bool result = vnl_matlab_read_or_die(matFile, M, varname.c_str());
        if(!result)
        {
            cout << "Error parsing file: " << endl << varname << endl << "from path:" << endl << path << endl;
        }
        else
        {
            cout << "Done" << endl;
            //Sanity check stats on input
            cout << "======Matrix Stats========" << endl;
            cout << "The matrix size is "  << M.rows() << " " << M.cols() << endl;
            cout << "Matrix mean is " << M.mean() << endl;
            cout << "Max entry is " << M.max_value() << endl;
            cout << "===============================" << endl << endl;
        }
    }
    else
    {
        cout << "Cannot open file: " << endl << filename << endl << "from path:" << endl << path << endl;
    }

}


//=====================================
//            Writing Out
//=====================================


void WriteToFile(const vnl_matrix<double> &M, string varname, string datasetName, string directory)
{
    cout << endl << "Writing " << varname << " to file...";
    vnl_matlab_filewrite FileOut((directory + datasetName + "_" + varname + ".mat").c_str());
    FileOut.write(M,varname.c_str());
    cout << "Done" << endl;            
}

void WriteToFile(const vnl_matrix<unsigned int> &M, string varname, string datasetName, string directory)
{
    cout << endl << "Writing " << varname << " to file...";
    vnl_matlab_filewrite FileOut((directory + datasetName + "_" + varname + ".mat").c_str());
    vnl_matrix<double> Md; 
    cast_vnl_matrix(M, Md);
    FileOut.write(Md,varname.c_str());
    cout << "Done" << endl;            
}

void WriteToFile(const vnl_vector<double> &M, string varname, string datasetName, string directory)
{
    cout << endl << "Writing " << varname << " to file...";
    vnl_matlab_filewrite FileOut((directory + datasetName + "_" + varname + ".mat").c_str());
    FileOut.write(M,varname.c_str());
    cout << "Done" << endl;            
}

void WriteToFile(const vnl_vector<int> &M, string varname, string datasetName, string directory)
{
    cout << endl << "Writing " << varname << " to file...";
    //For Matlab format, values must be double.
    //So, we'll cast the vector
    vnl_vector<double> Md(M.size());
    for(unsigned int i=0;i<M.size();++i)
        Md[i] = (double) M[i];

    vnl_matlab_filewrite FileOut((directory + datasetName + "_" + varname + ".mat").c_str());
    FileOut.write(Md,varname.c_str());
    cout << "Done" << endl;            
}

void WriteToFile(const vector<unsigned int> &V, string varname, string datasetName, string directory)
{
    vnl_vector<int> M(V.size());
    for(unsigned int i=0;i<V.size();++i)
    {
        M(i) = (int) V[i];
    }
    WriteToFile(M,varname,datasetName,directory);
}

void WriteToFile(const vector<double> &V, string varname, string datasetName, string directory)
{
    vnl_vector<double> M(V.size());
    for(unsigned int i=0;i<V.size();++i)
    {
        M(i) = V[i];
    }
    WriteToFile(M,varname,datasetName,directory);
}

//=====================================
//            Helper
//=====================================

void cast_vnl_matrix(const vnl_matrix<unsigned int> &S, vnl_matrix<double> &T)
{
    int r = S.rows();
    int c = S.cols();
    T.set_size(r,c);
    for(int i=0;i<r;++i)
    {
        for(int k=0;k<c;++k)
        {
            T(i,k) = (double)S(i,k);
        }
    }
}
