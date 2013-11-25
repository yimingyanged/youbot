#include "MJCMath.h"
#include <iostream>
#include <algorithm>
using namespace std;

namespace MJCMath
{
    void RandomKSubset(const unsigned int k, const unsigned int n, vector<unsigned int> &result)
    {
        //Returns a sorted random subset of size k from the range [0..n), in O(k) time
        //(make sure n<2^32 - otherwise unsigned int overflows)
        //The algorithm used is a simple modification of the Knuth shuffle
        //See http://www.techuser.net/randpermgen2.html for discussion
        //Also perhaps useful in related contexts is Conveyor Belt sampling, see:
        //http://www.maths.abdn.ac.uk/~igc/tch/mx4002/notes/node86.html

        if(result.size() != 0)
            cout << "ERROR: RandomKSubset() called with non-empty result container." << endl;
        result.reserve(k);
        boost::lagged_fibonacci607 rng(static_cast<unsigned int>(std::time(0)));

        if(k<=n)
        {
            boost::random_number_generator<boost::lagged_fibonacci607> random_less_than(rng); //Random numbers in range [0,n)
            std::map<unsigned int,unsigned int> range;

            for(unsigned int i=0;i<k;i++)
            {
                unsigned int r = i + random_less_than(n-i); //Random number in range [i,n)
                //Swap(i,r)
                if(range.find(i) == range.end())
                    range.insert(make_pair(i,i));
                if(range.find(r) == range.end())
                    range.insert(make_pair(r,r));
                //Do swap
                unsigned int temp = range[r];
                range[r] = range[i];
                range[i] = temp;
            }

            for(unsigned int i=0;i<k;i++)
            {
                if(range.find(i) == range.end())
                    cout << "ERROR: Looks like there is a bug in RandomKSubset()" << endl;

                result.push_back(range[i]);
            }
        }
        else
        {
            cout << "ERROR: RandomKSubset called with k>n" << endl;
        }
        std::sort(result.begin(),result.end());
    }

    //Variant of the above for performance critical code.
    //Uses internal random number generator, also removes error checking
    //Danger: Fails if n is greater than RAND_MAX, which for typical C++ implementations is only 32767
    //Non-fast version is good to 2^32.
    void RandomKSubset_Fast(const unsigned int k, const unsigned int n, vector<unsigned int> &result)
    {
        result.reserve(k);

        std::map<unsigned int,unsigned int> range;

        for(unsigned int i=0;i<k;i++)
        {
            unsigned int r = i +  (rand()%(n-i)); //Random number in range [i,n)
            //Swap(i,r)
            if(range.find(i) == range.end())
                range.insert(make_pair(i,i));
            if(range.find(r) == range.end())
                range.insert(make_pair(r,r));
            //Do swap
            unsigned int temp = range[r];
            range[r] = range[i];
            range[i] = temp;
        }

        for(unsigned int i=0;i<k;i++)
            result.push_back(range[i]);

        std::sort(result.begin(),result.end());
    }

    void RandomKSubset_Fast_NoSort(const unsigned int k, const unsigned int n, vector<unsigned int> &result)
    {
        result.reserve(k);

        std::map<unsigned int,unsigned int> range;

        for(unsigned int i=0;i<k;i++)
        {
            unsigned int r = i +  (rand()%(n-i)); //Random number in range [i,n)
            //Swap(i,r)
            if(range.find(i) == range.end())
                range.insert(make_pair(i,i));
            if(range.find(r) == range.end())
                range.insert(make_pair(r,r));
            //Do swap
            unsigned int temp = range[r];
            range[r] = range[i];
            range[i] = temp;
        }

        for(unsigned int i=0;i<k;i++)
            result.push_back(range[i]);
    }
}
