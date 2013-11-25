#ifndef WORDMAKER_INCLUDE_WORDMAKERDATATYPES_H_
#define WORDMAKER_INCLUDE_WORDMAKERDATATYPES_H_

#include <vector>
#include <set>
#include <map>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
using namespace std;


class PatchDescriptor
{
public:
    PatchDescriptor(unsigned int nRegionDescriptorSize,
                    unsigned int nFeatureDescriptorSize)
    {
        RegionDescriptor.set_size(nRegionDescriptorSize);
        RegionDescriptor.fill(0.0);
        FeatureDescriptor.set_size(nFeatureDescriptorSize);
        FeatureDescriptor.fill(0.0f);
    }

    vnl_vector<double> RegionDescriptor;
    vnl_vector<float> FeatureDescriptor;
};

// Stores info about the coordinates of an interest point, including which image
// part it's from for a multi-part image (e.g. stereo or Ladybug images may
// consist of multiple individual files).
class PatchLocation
{
public:
    PatchLocation(){}
    PatchLocation(unsigned int part, vnl_vector<double> RegionDescriptor)
    {
        nPartID = part;
        if(RegionDescriptor.size() == 6)
        {
           strength = RegionDescriptor[0];
           x = RegionDescriptor[1];
           y = RegionDescriptor[2];
           inv_scale = RegionDescriptor[3];
           skew = RegionDescriptor[4];
        }
        else if(RegionDescriptor.size() == 5)
        {
            //Old region descriptor, with no blob response information.
            x = RegionDescriptor[0];
            y = RegionDescriptor[1];
            inv_scale = RegionDescriptor[2];
            skew = RegionDescriptor[3];
            //Blob response info not present in file. Provide some fake value.
            strength = 42.0;
        }
        else
        {
            cerr << "ERROR: Trying to construct a patch location from an incomplete region descriptor" << endl;
        }
    }
    unsigned int nPartID;               //For a multi-part image, in which part is the patch.
    double strength;                    //blob response
    double x, y;
    double inv_scale;                   // 1/r^2
    double skew;                        // For eliptical regions. Zero for SURF.
};

typedef multimap<unsigned int,PatchLocation> BagOfRegions;

//Used for specifying regions of the image to exclude from processing.
struct ImageRegion
{
    double x_low;
    double x_high;
    double y_low;
    double y_high;
};


#endif  // WORDMAKER_INCLUDE_WORDMAKERDATATYPES_H_
