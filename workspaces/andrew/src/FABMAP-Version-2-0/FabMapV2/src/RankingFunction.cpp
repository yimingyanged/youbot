#include "RankingFunction.h"
#include "InnerLoop.h"

//********************************
//  Ranking Function Base Class
//********************************

RankingFunction::RankingFunction(const FabMapCalculator &i_world)
    :world(i_world)
{}

//********************************
//  FabMap Ranking (Base Class)
//********************************
FabMapRanking::FabMapRanking(const FabMapCalculator &i_world)
    :RankingFunction(i_world), m_nCurrentWordID(0), m_bPositive(true)
{}

void FabMapRanking::SetObservationComponent(const unsigned int nWordID,
                                            const unsigned int nNumInstancesOf_Word_InObservation,
                                            const unsigned int nNumInstancesOf_WordParent_InObservation)
{
    m_nCurrentWordID = nWordID;
    m_state_z = (nNumInstancesOf_Word_InObservation > 0);
    m_state_zp = (nNumInstancesOf_WordParent_InObservation > 0);

    //Set p(e=1 | z) for the current word
    //We're using naive-bayes update here. Slightly more sophistication is possible.
    p_e_1_given_z_1_L = world.GetP_e_1_given_z(nWordID,1);
    p_e_1_given_z_0_L = world.GetP_e_1_given_z(nWordID,0);

    //Update Ks for new wordID.
    SetEquationConstants();
}

//Are we dealing with positive observations (i.e, present in the observation, and in the scene in the map)
//or negative observations (absent in the observation, but present in the scene in the map)
void FabMapRanking::ProcessPositiveObservations(const bool bPositive)
{
    m_bPositive = bPositive;
    SetEquationConstants();     //Update Ks for new observation mode.
}

double FabMapRanking::GetVote(const unsigned int nNumInstancesOf_Word_InScene) const
{
    //p(z=s|L) for Naive Bayes
    //
    // p(z=s|L) = p(z=s | e=1)p(e=1 | L) + p(z=s | e=0)p(e=0 | L)
    //
    //Define K1 = p(z=s | e=1), K2 = p(z=s | e=0)
    //Manipulate a bit for:
    //
    //p(z=s|L) = (K1 - K2)p(e=1 | L) + K2
    //
    //Now, our vote will be
    //   log( p(z=s|L_where_word_was_present_last_time)/p(z=s|L_where_word_was_absent_last_time) )
    //This is because, (with the inverted index), we cast votes only for those places where the word was present last time
    //So the likelihood is p(z=s|L_where_word_was_present_last_time)
    //However, all the other places for which we don't cast votes have likelihood p(z=s|L_where_word_was_absent_last_time)
    //So, by casting the above vote, we get the same distribution, while only having to do calculations for places where the word was observed.
    return log( (K1_minus_K2*p_e_1_given_z_1_L + K2) / (K1_minus_K2*p_e_1_given_z_0_L + K2) );
}

//********************************
//      Naive Bayes FabMap
//********************************
NaiveBayesLikelihood::NaiveBayesLikelihood(const FabMapCalculator &i_world)
    :FabMapRanking(i_world)
{}

void NaiveBayesLikelihood::SetEquationConstants()
{
    if(m_bPositive)
    {
        //p(z=1|e=1)
        K1 = world.p_z_1_given_e_1[m_nCurrentWordID];
        //p(z=1|e=0)
        K2 = world.p_z_1_given_e_0[m_nCurrentWordID];
    }
    else
    {
        //p(z=0|e=1)
        K1 = 1.0 - world.p_z_1_given_e_1[m_nCurrentWordID];
        //p(z=0|e=0);
        K2 = 1.0 - world.p_z_1_given_e_0[m_nCurrentWordID];
    }
    K1_minus_K2 = K1 - K2;
}

string NaiveBayesLikelihood::DescribeRankingFunction() const
{
    return "FABMAP_NAIVE_BAYES";
}


//********************************
//       Chow Liu FabMap
//********************************
ChowLiuLikelihood::ChowLiuLikelihood(const FabMapCalculator &i_world)
    :FabMapRanking(i_world)
{}

void ChowLiuLikelihood::SetEquationConstants()
{
    if(m_bPositive)
    {
        //p(z=1|e=1, zp)
        K1 = world.GetP_z_Given_Zp_and_E(m_nCurrentWordID,1,1,m_state_zp);
        //p(z=1|e=0, zp)
        K2 = world.GetP_z_Given_Zp_and_E(m_nCurrentWordID,1,0,m_state_zp);
    }
    else
    {
        //p(z=0|e=1, zp)
        K1 = world.GetP_z_Given_Zp_and_E(m_nCurrentWordID,0,1,m_state_zp);
        //p(z=0 | e =0, zp);
        K2 = world.GetP_z_Given_Zp_and_E(m_nCurrentWordID,0,0,m_state_zp);
    }
    K1_minus_K2 = K1 - K2;
}

string ChowLiuLikelihood::DescribeRankingFunction() const
{
    return "FABMAP_CHOW_LIU";
}
