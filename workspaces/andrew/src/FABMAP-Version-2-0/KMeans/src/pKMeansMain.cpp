#include "pKMeans.h"
#include "mpi_kmeans.h"
#pragma warning(disable: 4996) //warnings for non-secure functions

int main(int argc,char* argv[])
{
    const char * sMissionFile = "Config.moos";

    switch(argc)
    {
    case 2:
        sMissionFile = argv[1];
    }

     KMeans clusterer(sMissionFile);
     clusterer.Run();
     return 0;
}




