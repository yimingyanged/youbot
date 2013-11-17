#ifndef SAMPLER_H
#define SAMPLER_H 1

#include "FabMap.h"
#include <iterator>

//Iterator for returning data from samplers
//samplers contain an internal buffer called m_samples
//and define a traversal order over part/all of that buffer via m_TraversalOrder
//SampleIterator returns the samples in that order
class SampleIterator : public std::iterator <std::random_access_iterator_tag,SceneRecord>
{
public:
    //constructor
    SampleIterator(const SceneRecordConatiner &i_samples,const vector<unsigned int> &i_TraversalOrder, const unsigned int nInitialPos = 0)
        : m_samples(i_samples), m_TraversalOrder(i_TraversalOrder), pos(nInitialPos)
    {}

    SampleIterator(const SampleIterator &other)
        : m_samples(other.m_samples), m_TraversalOrder(other.m_TraversalOrder), pos(other.pos)
    {}

    //Increment operator
    void operator++ () { pos++; }
    void operator-- () { pos--; }

    void operator+= (const unsigned int inc) { pos += inc; }
    void operator-= (const unsigned int inc) { pos -= inc; }   

    //Random access
    SampleIterator& operator () (const unsigned int newpos) { pos = newpos; return *this; }  

    const SceneRecord& operator* () const { return m_samples[m_TraversalOrder[pos]]; }

    bool operator== (const SampleIterator &other) const { return pos == other.pos; }
    bool operator!= (const SampleIterator &other) const { return pos != other.pos; }

private:   
    const SceneRecordConatiner &m_samples;
    const vector<unsigned int> &m_TraversalOrder;
    unsigned int pos;

    //Disallow use of assignment operator. Not valid for class with reference members.
    void operator= (const SampleIterator &other);
};

//Abstract base class for samplers
class Sampler
{
    friend class SampleIterator;
public:
    Sampler(FabMapCalculator &i_world);
    virtual ~Sampler() {}

    virtual void DrawNSamples(const unsigned int n) = 0;    //After calling this, the internal sample buffer is refreshed with N samples. m_SamplesInvIndex defines their inverted index.

    SampleIterator begin();
    SampleIterator end();
    
    InvertedIndex m_SamplesInvIndex;
    vector<double> m_SamplesNegativeBaseline;               //Cached negative votes for each sample.
    
    //And some info functions
    virtual string DescribeSampler() const = 0;
    virtual bool FiniteSampler() const { return false;}    //Returns true is the sampler can generate only a finite number of unique samplers, e.g. like the RealSampler.
protected:
    FabMapCalculator &world;            //For the global parameters, e.g. CL tree
    SceneRecordConatiner m_Samples;       //Output samples appear here.
    vector<unsigned int> m_TraversalOrder;//Defines a view of the m_Samples vector. Iterator returns m_Samples listed in m_TraversalOrder
    void BuildInvertedIndexForSamples();
    void UpdateNegativeBaseline();
};

//This class implements the mean field approximation
//It's not really a sampler, but it's convenient to implement it using this interface
//It returns only one sample, which is the average place
class MeanFieldApprox : public Sampler
{
public:
    MeanFieldApprox(FabMapCalculator &i_world);
    ~MeanFieldApprox() {};
    virtual void DrawNSamples(const unsigned int n);

    string DescribeSampler() const;
    unsigned int GetMaxUniqueSamples() const {return 1;}
    bool FiniteSampler() const { return true;}
private:
    void OopsMessage() const;
};

//This class returns no samples
//Use this when you want to get an ML decision over the places in the map, with no sampling set.
class NullSampler : public Sampler
{
public:
    NullSampler(FabMapCalculator &i_world);
    ~NullSampler() {};
    virtual void DrawNSamples(const unsigned int n);

    string DescribeSampler() const;
    unsigned int GetMaxUniqueSamples() const {return 0;}
    bool FiniteSampler() const { return true;}
};

//This class uses the images in the vocabulary directory as RealSamples (tm).
class RealSampler : public Sampler
{
public:
    RealSampler(FabMapCalculator &i_world,string i_sSamplesPath,string i_sSamplesFile);        
    RealSampler(FabMapCalculator &i_world);
    ~RealSampler() {};
    void LoadSamples();
    virtual void DrawNSamples(const unsigned int n);

    string DescribeSampler() const;
    unsigned int GetMaxUniqueSamples() const {return m_nNumSamples;}
    bool FiniteSampler() const { return true;}    

private:
    unsigned int m_nNumSamples;
    bool m_bLastRequestWasForAllSamples;
    string m_sSamplesPath,m_sSamplesFile;
};

#endif //SAMPLER_H
