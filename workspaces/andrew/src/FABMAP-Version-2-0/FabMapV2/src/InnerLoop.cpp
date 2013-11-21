#include "FabMap.h"
#include "InnerLoop.h"
#include "Sampler.h"
#include "RankingFunction.h"
#include "MJCMath.h"

void FabMapCalculator::CalculateAppearanceLikelihoods(vnl_vector<double> &log_p_obs_given_scene,
                                                      const SceneRecord &observation,
                                                      const unsigned int nIndexOfFirstSample,
                                                      const InvertedIndex &SamplesInvIndex,
                                                      const vector<double> &SamplesNegBaseline,
                                                      const SampleIterator &sbegin,
                                                      const SampleIterator &send) const
{
    //Places are initialized such that their likelihood corresponds to all words being observed in state (z=0|zp=0)
    //Given an observation we need to update positive observations p(z=1 | zp = s)
    //and Markov Blanket observations p(z=0 | zp = 1)
    const unsigned int nNumScenes = m_SceneRecordsInMap.size();
    const unsigned int nNumSamples = SamplesNegBaseline.size();
    
    for(unsigned int i=0;i<nNumScenes;i++)
        log_p_obs_given_scene(i) = m_NegativeBaseline[i];

    for(unsigned int i=0;i<nNumSamples;i++)
        log_p_obs_given_scene(nIndexOfFirstSample+i) = SamplesNegBaseline[i];

    //Now, update the likelihoods using the observation.
    //First for the Markov Blanket of the observation, (z=0 | zp = 1)
    vector<double> CompositeVote;
    vector<unsigned int> MarkovBlanket;
    GetMarkovBlanket(observation,MarkovBlanket);
    GetCompositeVoteRelativeToNegBaseline(MarkovBlanket,CompositeVote);
    const unsigned int nNumWords = MarkovBlanket.size();
    for(unsigned int i = 0;i<nNumWords;++i)
    {
        const unsigned int nWordID =  MarkovBlanket[i];
        const double dfVote = CompositeVote[i];
        //Add the right vote to each scene it occurred in
        const unsigned int nNumScenesInWhichObserved = m_WordToScenes[nWordID].size();
        for(unsigned int j=0;j<nNumScenesInWhichObserved;++j)
        {
            log_p_obs_given_scene(m_WordToScenes[nWordID][j].nSceneID) += dfVote;
        }

        //And for each sample
        const unsigned int nNumSamplesInWhichObserved = SamplesInvIndex[nWordID].size();
        for(unsigned int j=0;j<nNumSamplesInWhichObserved;++j)
        {
            log_p_obs_given_scene(nIndexOfFirstSample+SamplesInvIndex[nWordID][j].nSceneID) += dfVote;
        }
    }

    //Next update the likelihood for the observed words, i.e.
    //(z=1 | zp =1) or (z=1 | zp =0)
    GetCompositeVoteRelativeToNegBaseline(observation,CompositeVote);
    const unsigned int nWordsObserved = observation.nNumWords;
    for(unsigned int i = 0;i<nWordsObserved;++i)
    {
        const unsigned int nWordID =  observation.WordData[i].nID;
        const double dfVote = CompositeVote[i];
        //Add the right vote to each scene it occured it
        const unsigned int nNumScenesInWhichObserved = m_WordToScenes[nWordID].size();
        for(unsigned int j=0;j<nNumScenesInWhichObserved;++j)
        {
            log_p_obs_given_scene(m_WordToScenes[nWordID][j].nSceneID) += dfVote;
        }

        //And for each sample
        const unsigned int nNumSamplesInWhichObserved = SamplesInvIndex[nWordID].size();
        for(unsigned int j=0;j<nNumSamplesInWhichObserved;++j)
        {
            log_p_obs_given_scene(nIndexOfFirstSample+SamplesInvIndex[nWordID][j].nSceneID) += dfVote;
        }
    }

}

void FabMapCalculator::GetCompositeVoteRelativeToNegBaseline(const SceneRecord &observation, vector<double> &ReturnedVotes) const
{
    //Vote to update a place that was initialized to the "negative baseline" - i.e. assuming all words in scene not observed
    //If the word is observed, the new vote is -NegativeVote + PositiveVote
    vector<double> PositiveVotes;
    bool bPositiveObservation = true;
    CalculateVotes(observation,bPositiveObservation,PositiveVotes);

    vector<double> NegativeVotes;
    CalculateVotes_NegativeBaseline(observation,NegativeVotes);

    const unsigned int nNumVotes = PositiveVotes.size();
    ReturnedVotes.assign(nNumVotes,0.0);
    for(unsigned int i=0;i<nNumVotes;i++)
        ReturnedVotes[i] = PositiveVotes[i] - NegativeVotes[i];
}

void FabMapCalculator::GetCompositeVoteRelativeToNegBaseline(const vector<unsigned int> &MarkovBlanket, vector<double> &ReturnedVotes) const
{
    //Vote to update a place that was initialized to the "negative baseline" - i.e. assuming all words in scene not observed
    //If the word is observed, the new vote is -NegativeVote + PositiveVote
    vector<double> PositiveVotes;
    CalculateVotes_MarkovBlanket(MarkovBlanket,PositiveVotes);

    vector<double> NegativeVotes;
    CalculateVotes_NegativeBaseline(MarkovBlanket,NegativeVotes);

    const unsigned int nNumVotes = PositiveVotes.size();
    ReturnedVotes.assign(nNumVotes,0.0);
    for(unsigned int i=0;i<nNumVotes;i++)
        ReturnedVotes[i] = PositiveVotes[i] - NegativeVotes[i];
}

double FabMapCalculator::GetNegativeBaselineForScene(const SceneRecord &observation) const
{
    vector<double> Votes;
    CalculateVotes_NegativeBaseline(observation,Votes);
    
    double dfTotal = 0.0;
    const unsigned int nNumVotes = Votes.size();
    for(unsigned int i=0;i<nNumVotes;i++)
        dfTotal += Votes[i];

    return dfTotal;
}

void FabMapCalculator::CalculateVotes(const SceneRecord &observation, bool bPositiveObservation, vector<double> &ReturnedVotes) const
{
    //Given an observation, return the positive vote for each word in the observation
    mp_RankingFunction->ProcessPositiveObservations(bPositiveObservation);
    const unsigned int nObs = bPositiveObservation ? 1 : 0;

    const unsigned int nWordsObserved = observation.nNumWords;
    ReturnedVotes.assign(nWordsObserved,0.0);
    for(unsigned int i = 0;i<nWordsObserved;++i)
    {
        const unsigned int nWordID =  observation.WordData[i].nID;
        if(Marginals[nWordID] != 0.0)
        {
            const unsigned int state_z = bPositiveObservation ? observation.WordData[i].nN : 0;
            const int parent_id = Parent(nWordID);
            const unsigned int state_zp  = std::binary_search(observation.WordData.begin(),observation.WordData.end(),WordWithInterestPoints(parent_id)) 
                                           ? 1 : 0;

            mp_RankingFunction->SetObservationComponent(nWordID,state_z,state_zp);

            ReturnedVotes[i] = mp_RankingFunction->GetVote(nObs);  //Restrictive assumption. Positive votes do not vary by place. In practice, we always had this anyway. //So 1 rather than (m_WordToScenes[nWordID][j].nN);
        }
    }
}

void FabMapCalculator::CalculateVotes_MarkovBlanket(const vector<unsigned int> &MarkovBlanket, vector<double> &ReturnedVotes) const
{
    //For a set of words with p(z=0|zp=1), return the appropriate votes
    const unsigned int nObs = 0;
    const unsigned int state_z = 0;
    const unsigned int state_zp = 1;

    const unsigned int nNumWords = MarkovBlanket.size();
    ReturnedVotes.assign(nNumWords,0.0);
    for(unsigned int i = 0;i<nNumWords;++i)
    {
        const unsigned int nWordID =  MarkovBlanket[i];
        if(Marginals[nWordID] != 0.0)
        {
            mp_RankingFunction->SetObservationComponent(nWordID,state_z,state_zp);
            ReturnedVotes[i] = mp_RankingFunction->GetVote(nObs);
        }
    }
}

void FabMapCalculator::CalculateVotes_NegativeBaseline(const SceneRecord &observation, vector<double> &ReturnedVotes) const
{
    //Vote if all words in the scene are not observed
    //i.e p(z=0|z=0)
    mp_RankingFunction->ProcessPositiveObservations(false);
    const unsigned int state_z = 0;
    const unsigned int state_zp = 0;

    const unsigned int nWordsObserved = observation.nNumWords;
    ReturnedVotes.assign(nWordsObserved,0.0);
    for(unsigned int i = 0;i<nWordsObserved;++i)
    {
        const unsigned int nWordID =  observation.WordData[i].nID;
        if(Marginals[nWordID] != 0.0)
        {           
            mp_RankingFunction->SetObservationComponent(nWordID,state_z,state_zp);
            ReturnedVotes[i] = mp_RankingFunction->GetVote(0);
        }
    }
}

void FabMapCalculator::CalculateVotes_NegativeBaseline(vector<unsigned int> &WordIDs, vector<double> &ReturnedVotes) const
{
    //Vote if all words in the scene are not observed
    //i.e p(z=0|z=0)
    mp_RankingFunction->ProcessPositiveObservations(false);
    const unsigned int state_z = 0;
    const unsigned int state_zp = 0;

    const unsigned int nWordsObserved = WordIDs.size();
    ReturnedVotes.assign(nWordsObserved,0.0);
    for(unsigned int i = 0;i<nWordsObserved;++i)
    {
        const unsigned int nWordID =  WordIDs[i];
        if(Marginals[nWordID] != 0.0)
        {           
            mp_RankingFunction->SetObservationComponent(nWordID,state_z,state_zp);
            ReturnedVotes[i] = mp_RankingFunction->GetVote(0);
        }
    }
}

void FabMapCalculator::GetMarkovBlanket(const SceneRecord &observation,vector<unsigned int> &MarkovBlanket) const
{
    //Here we compute a list of those words which weren't observed in the current observation, but whose parent in the CL tree was observed.
    //This is (almost) the Markov Blanket as defined by the Chow Liu tree. (Almost because it doesn't include the parents of observed words).
    MarkovBlanket.clear();

    for(unsigned int i=0; i<observation.nNumWords;++i)
    {
        const unsigned int nWordID =  observation.WordData[i].nID;
        const unsigned int nNumChildren = Children[nWordID].size(); 
        for(unsigned int j = 0;j<nNumChildren;++j)
        {           
            const unsigned int child_id = Children[nWordID][j];

            //If the child is not present in the observation, add it to the Markov blanket.
            if(!std::binary_search(observation.WordData.begin(),observation.WordData.end(),WordWithInterestPoints(child_id)))
                MarkovBlanket.push_back(child_id);
        }
    }

    //Needs to be sorted, because we'll run binary searches on it, etc.
    std::sort(MarkovBlanket.begin(),MarkovBlanket.end());
}

