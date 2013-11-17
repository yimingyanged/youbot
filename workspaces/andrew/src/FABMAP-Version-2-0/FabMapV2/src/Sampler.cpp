#include "Sampler.h"
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <sstream>
#include "MJCMath.h"

//********************************
//        Sampler Base Class
//********************************

Sampler::Sampler(FabMapCalculator &i_world)
    :world(i_world)
{}

SampleIterator Sampler::begin()
{
    return SampleIterator(m_Samples,m_TraversalOrder,0);
}

SampleIterator Sampler::end()
{
    return SampleIterator(m_Samples,m_TraversalOrder,m_TraversalOrder.size());
}

void Sampler::BuildInvertedIndexForSamples()
{
    m_SamplesInvIndex.clear();
    m_SamplesInvIndex = InvertedIndex(world.m_nVocabSize,vector<InvertedIndexEntry>());

    for(unsigned int i=0;i<m_TraversalOrder.size();i++)
    {
        const unsigned int nCurrentSampleID = i;
        const unsigned int nInternalBufferIndex = m_TraversalOrder[i];
        const unsigned int nWordsInScene = m_Samples[nInternalBufferIndex].nNumWords;
        for(unsigned int j=0;j<nWordsInScene;j++)
        {
            InvertedIndexEntry WordSighting;
            WordSighting.nSceneID = nCurrentSampleID;
            WordSighting.nN = m_Samples[nInternalBufferIndex].WordData[j].nN;
            m_SamplesInvIndex[m_Samples[nInternalBufferIndex].WordData[j].nID].push_back(WordSighting);
        }
    }

    //Whenever we rebuild the inverted index, we will also want to update the negative baseline votes
    UpdateNegativeBaseline();
}

void Sampler::UpdateNegativeBaseline()
{
    m_SamplesNegativeBaseline.assign(m_TraversalOrder.size(),0.0) ;
    for(unsigned int i=0;i<m_TraversalOrder.size();i++)
    {
        const unsigned int nCurrentSampleID = i;
        const unsigned int nInternalBufferIndex = m_TraversalOrder[i];
        m_SamplesNegativeBaseline[nCurrentSampleID] = world.GetNegativeBaselineForScene(m_Samples[nInternalBufferIndex]);
    }
}


//********************************
//         Mean Field
//********************************

MeanFieldApprox::MeanFieldApprox(FabMapCalculator &i_world)
    :Sampler(i_world)
{
    //Not implemented yet!
    OopsMessage();
    //Ensure containers are empty (this is slightly paranoid...)
    m_Samples.clear();
    m_TraversalOrder.clear();
    BuildInvertedIndexForSamples();
}

void MeanFieldApprox::DrawNSamples(unsigned int n)
{
    if(n != 1)
    {
        cout << "ERROR. MeanFieldSampler received a request for more samples than it can provide." << endl;
        cout << " N_requested: " << n << endl;
        cout << " N_available: " << 1 << endl;
    }
    else
    {
        //Not implemented yet!
        OopsMessage();
    }

}

void MeanFieldApprox::OopsMessage() const
{
    cout << "ERROR: MeanField sampler not implemented in FabMap 2.0!"
         << " Not clear how it should be implemented when places no longer have continuous-valued word membership.";
}

string MeanFieldApprox::DescribeSampler() const
{
    return "MEAN FIELD"; 
}

//********************************
//         Null Sampler
//********************************

NullSampler::NullSampler(FabMapCalculator &i_world)
    :Sampler(i_world)
{
    //Ensure containers are empty (this is slightly paranoid...)
    m_Samples.clear();
    m_TraversalOrder.clear();
    BuildInvertedIndexForSamples();
}

void NullSampler::DrawNSamples(unsigned int n)
{
    //Do nothing
    if(n != 0)
    {
        cout << "ERROR. NullSampler received a request for more samples than it can provide." << endl;
        cout << " N_requested: " << n << endl;
        cout << " N_available: " << 0 << endl;
    }
}

string NullSampler::DescribeSampler() const
{
    return "NULL SAMPLER"; 
}

//********************************
//            Real Sampler
//********************************
RealSampler::RealSampler(FabMapCalculator &i_world,string i_sSamplesPath,string i_sSamplesFile)
            :Sampler(i_world)
{
    cout << "Loading alternate real samples." << endl;
    m_sSamplesPath = i_sSamplesPath;
    m_sSamplesFile = i_sSamplesFile;
    LoadSamples();
}

RealSampler::RealSampler(FabMapCalculator &i_world)
            :Sampler(i_world)
{
    //Default to using the vocabulary training images as the set of samples
    m_sSamplesPath = world.m_sVocabPath ;
    m_sSamplesFile = world.m_sVocabName + ".oxs";
    LoadSamples();
}

void RealSampler::LoadSamples()
{
   //First, a sanity check
   unsigned int num_scenes, vocab_size;
   ParseOXS_PeekDimensions(m_sSamplesPath,m_sSamplesFile,num_scenes,vocab_size);
   if(vocab_size != world.m_nVocabSize)
   {
        cerr << endl << " ERROR in RealSampler! The sampling set specified does not match the vocabulary!" << endl;
        //This is such a terrible thing that we should quit now.
        exit(0);
   }

   //Now, load the samples
   ParseOXS(m_sSamplesPath,m_sSamplesFile,m_Samples,false,world.m_dfBlobResponseThreshold);
   m_nNumSamples = m_Samples.size();

   if(m_nNumSamples==0)
        cout << endl << "Failed to create Sampler. Couldn't find vocab scenes file at path:" << endl << world.m_sVocabPath << world.m_sVocabName << ".oxs" << endl << endl;

    //Call DrawNSamples to build InvertedIndex
    m_bLastRequestWasForAllSamples = false;
    DrawNSamples(m_nNumSamples); 
}

void RealSampler::DrawNSamples(const unsigned int n)
{
    if(n > m_nNumSamples)
    {
        cout << "ERROR. RealSampler received a request for more samples than it can provide." << endl;
        cout << " N_requested: " << n << endl;
        cout << " N_available: " << m_nNumSamples << endl;
    }
    else if(n == m_nNumSamples && m_bLastRequestWasForAllSamples)
    {
        //Index and InvertedIndex remain unchanged
        //Nothing to do
    }
    else if(n == m_nNumSamples)
    {
        m_TraversalOrder.clear();
        for(unsigned int i=0; i<n; i++)
            m_TraversalOrder.push_back(i);

        //Rebuild inverted index
        BuildInvertedIndexForSamples();

        m_bLastRequestWasForAllSamples = true;
    }
    else
    {
        //Pick a random subset of samples
        m_TraversalOrder.clear();
        MJCMath::RandomKSubset(n,m_nNumSamples,m_TraversalOrder);

        //Rebuild inverted index
        BuildInvertedIndexForSamples();

        m_bLastRequestWasForAllSamples = false;
    }
}

string RealSampler::DescribeSampler() const
{
    return "REAL SAMPLES"; 
}
