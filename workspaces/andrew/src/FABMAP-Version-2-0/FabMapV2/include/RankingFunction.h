#include "FabMap.h"

class RankingFunction
{
public:
    RankingFunction(const FabMapCalculator &i_world);
    virtual ~RankingFunction() {}

    virtual void SetObservationComponent(const unsigned int nWordID,
                                         const unsigned int nNumInstancesOf_Word_InObservation,
                                         const unsigned int nNumInstancesOf_WordParent_InObservation) = 0;
    virtual void ProcessPositiveObservations(const bool bPositive) =0;

    virtual double GetVote(const unsigned int nNumInstancesOf_Word_InScene) const = 0;

    virtual string DescribeRankingFunction() const = 0;
protected:
    const FabMapCalculator &world;
};

//Base class for FabMap ranking functions
class FabMapRanking : public RankingFunction
{
public:
    FabMapRanking(const FabMapCalculator &i_world);
    virtual ~FabMapRanking() {}
    
    virtual void SetObservationComponent(const unsigned int nWordID,
                                         const unsigned int nNumInstancesOf_Word_InObservation,
                                         const unsigned int nNumInstancesOf_WordParent_InObservation);

    virtual void ProcessPositiveObservations(const bool bPositive);

    //Return likelihood of place L, which contains nNumInstancesOf_Word_InScene of m_nCurrentWordID.
    virtual double GetVote(const unsigned int nNumInstancesOf_Word_InScene) const;

    virtual string DescribeRankingFunction() const = 0;
protected:
    virtual void SetEquationConstants() =0;
    
    unsigned int m_nCurrentWordID;
    bool m_bPositive;                //Are we currently dealing with positive (z=1) or negative (z=0) observations?
    
    unsigned int m_state_z;
    unsigned int m_state_zp;

    double p_e_1_given_z_1_L;        //p(e=1 | z=1), for the current word. This is the probability that the word exists at a place, p(e=1|L), given that we have seen it there exactly once before. Because our detector model is currently independent of location, this value is the same for all places that the word was observed.
    double p_e_1_given_z_0_L;        //p(e=1 | z=0), for the current word. Probability that word exists, given that we have exactly one observation from the place, in which we did not see the word.

    double K1, K2;
    double K1_minus_K2;
};

//Naive Bayes FabMap
class NaiveBayesLikelihood : public FabMapRanking
{
public:
    NaiveBayesLikelihood(const FabMapCalculator &i_world);
    virtual ~NaiveBayesLikelihood() {}
    string DescribeRankingFunction() const;

private:
    virtual void SetEquationConstants();
};

//Chow Liu FabMap
class ChowLiuLikelihood : public FabMapRanking
{
public:
    ChowLiuLikelihood(const FabMapCalculator &i_world);
    virtual ~ChowLiuLikelihood() {}
    string DescribeRankingFunction() const;
private:
    virtual void SetEquationConstants();
};

