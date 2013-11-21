#ifndef MJCMATH_H
#define MJCMATH_H 1

#include <boost/random.hpp>
#include <map>
#include <ctime>
#include <vector>

namespace MJCMath
{
    //Returns a sorted random subset of size k from the range [0..n), in O(k) time
    //(make sure n<2^32 - otherwise unsigned int overflows)
    //The algorithm used is a simple modification of the Knuth shuffle
    //See http://www.techuser.net/randpermgen2.html for discussion
    //Also perhaps useful in related contexts is Conveyor Belt sampling, see:
    //http://www.maths.abdn.ac.uk/~igc/tch/mx4002/notes/node86.html

    void RandomKSubset(const unsigned int k, const unsigned int n, std::vector<unsigned int> &result);

    //Variant of the above for performance critical code.
    //Removes error checking, uses an externally provided radom number generator, so doesn't initialize one again each time.
    //About 4x faster than vanilla version.
    //Typical call time about k microseconds. NoSort perhaps 10% faster.
    void RandomKSubset_Fast(const unsigned int k, const unsigned int n, std::vector<unsigned int> &result);
    void RandomKSubset_Fast_NoSort(const unsigned int k, const unsigned int n, std::vector<unsigned int> &result);
}
#endif //MJCMATH_H
