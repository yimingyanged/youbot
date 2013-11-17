#include "LocationPrior.h"
#include "config.h"
#include <iostream>
#include <sstream>
using namespace std;

//*****************************************
//        Prior Provider Base Class
//*****************************************
LocationPriorProvider::LocationPriorProvider(double prob_NewPlace)
:
p_at_new_place(prob_NewPlace)
{}

//**********************************************
//       Prior Modifier - Exclude Local Matches
//**********************************************
// This function takes a prior, and alters it 
// so that the prior is 0 on the last k places.
// There's no point comparing to these recently acquired places.
// Odometry is good in this region.
// Without this there is a tendency to match these recent observations, because often views overlap.
vector<double> LocationPriorProvider::ExcludeLocalMatches(const vector<double> &existing_prior,
                                                          unsigned int k) const
{
    //Input k is the "exclusion zone". i.e. matches between [s-k:s-1] are disallowed.
    //s is the number of existing places in the map.
    //Our prior will be over places [0,s]. i.e - Each existing place, and the new place at index s.
    const unsigned int s = existing_prior.size()-1;
    vector<double> prior(existing_prior);

    if(k>s)
    {
        k = s;
    }

    //Set the prior on the last k places to be zero.
    double p_mass = 0.0;
    for(unsigned int i=1; i<=k; i++)
    {
        p_mass += prior[s-i];
        prior[s-i] = 0;
    }

    //Now, renormalize the prior.
    double normalization = 1.0 - p_mass;
    for(unsigned int i=0; i<=s; i++)
    {
        prior[i] /= normalization;
    }

    return prior;
}

vector<double> LocationPriorProvider::VNLToSTL(const vnl_vector<double> &v)
{
    return vector<double>(v.begin(),v.end());
}

//******************************
//        Uniform Prior
//******************************
UniformLocationPrior::UniformLocationPrior(double prob_NewPlace)
:
LocationPriorProvider(prob_NewPlace)
{}

vector<double> UniformLocationPrior::GetLocationPrior(const LocationProbabilityContainer &location_probability) const
{
    const unsigned int s = location_probability.size(); 
    //s is the number of existing places in the map.
    //Our prior will be over places [0,s]. i.e - Each existing place, and the new place at index s.

    //Assign uniform probability to all existing places. New place probability determined by p_at_new_place
    vnl_vector<double> prior(s+1,(1.0/((double)s)));
    prior *= (1.0-p_at_new_place);
    prior(s) = p_at_new_place;

    return VNLToSTL(prior);
}

string UniformLocationPrior::DescribePrior() const
{
    return "Prior: UNIFORM\n"; 
}

//**********************************************
//        LeftRight Motion Model
//**********************************************
LeftRightMotionModelPrior::LeftRightMotionModelPrior(double prob_NewPlace)
:
LocationPriorProvider(prob_NewPlace),
m_dfBias(0.5),
m_dfSmoothingFactor(0.2)
{}

LeftRightMotionModelPrior::LeftRightMotionModelPrior(double prob_NewPlace,double motion_bias)    
:
LocationPriorProvider(prob_NewPlace),
m_dfBias(motion_bias),
m_dfSmoothingFactor(0.2) // Reduce this value to make the motion model more strict. e.g. as low as (0.00001) might be reasonable if you really believe the motion model!
                         // Essentially this value controls the relative strength of the prior vs the appearance likelihood.
{}

vector<double> LeftRightMotionModelPrior::GetLocationPrior(const LocationProbabilityContainer &location_probability) const
{
    const unsigned int s = location_probability.size(); 
    //s is the number of existing places in the map.
    //Our prior will be over places [0,s]. i.e - Each existing place, and the new place at index s.

    vnl_vector<double> prior(s+1,0.0);
    
    prior(1) += location_probability.at(0);

    //Now, each place spreads it's probability to its left and right neighbours
    double fraction_forward = m_dfBias;
    double fraction_backward = 1.0 - m_dfBias;

    for(unsigned int i=1;i<s-1;++i)
    {
        prior(i-1) += location_probability[i]*fraction_backward;
        prior(i+1) += location_probability[i]*fraction_forward;
    }
    if(s!=1)
    {
        //Probability going forward from the last place is split up as:
        // (probability mass)*p_at_new_place     is assigned to the new place
        // (probability mass)*(1-p_at_new_place) is spread evenly over all known places
        // If we had a valency estimate for each known place, then we could improve this prior.
        prior(s-2) += location_probability[s-1]*fraction_backward;
    
        //Spread (1-p_at_new_place)*p evenly over the s known places
        double pmass_to_each_known_place = (1-p_at_new_place)*location_probability[s-1]*fraction_forward*(1/(double)s);
        for(unsigned int i = 0; i<s;++i)
        {
            prior(i) += pmass_to_each_known_place;    
        }
        //Now, assign (p_at_new_place)*p to the new place
        prior(s) += p_at_new_place*location_probability[s-1]*fraction_forward;
    }

    //Assign a small baseline probability to all cells
    //We're generating the prior in a fairly crude way. We don't want it to be too confident.
    prior *= (1-m_dfSmoothingFactor);
    double pmass_teleport = m_dfSmoothingFactor/((double)s+1);
    for(unsigned int i = 0; i<(s+1);++i)
    {
        prior(i) += pmass_teleport;    
    }

    return VNLToSTL(prior);
}

string LeftRightMotionModelPrior::DescribePrior() const
{
    if(m_dfBias == 0.5)
    {
        return "Prior: Previous Position + Motion Model\nMotion Model: Always Move\n";
    }
    else if (m_dfBias > 0.5)
    {
        std::stringstream ss;
        ss << "Prior: Previous Position + Motion Model" << std::endl 
            << "Motion Model: Always Move, Bias Forward (" << m_dfBias << ")" << std::endl;
        return ss.str();
    }
    else
    {
        std::stringstream ss;
        ss << "Prior: Previous Position + Motion Model" << std::endl 
            << "Motion Model: Always Move, Bias Backward (" << m_dfBias << ")" << std::endl;
        return ss.str();
    }
}

//**********************************************
//        Prior via Move-or-Stay Motion Model
//**********************************************
LeftRightBelowMotionModelPrior::LeftRightBelowMotionModelPrior(double prob_NewPlace)
:
LocationPriorProvider(prob_NewPlace),
m_dfSmoothingFactor(0.2)
{}

vector<double> LeftRightBelowMotionModelPrior::GetLocationPrior(const LocationProbabilityContainer &location_probability) const
{
    //s is the number of existing places in the map.
    //Our prior will be over places [0,s]. i.e - Each existing place, and the new place at index s.
    const unsigned int s = location_probability.size(); 

    //Calculate the prior on each known location given the previous estimate of position, using our really silly motion model. 
    //Assume that actions reverse, stay, go forward are equally likely
    vnl_vector<double> prior(s+1,0.0);
    
    prior(0) += location_probability.at(0)/2;
    prior(1) += location_probability.at(0)/2;
    //Now, each place spreads it's probability equally to itself, and it's left and right neighbours
    for(unsigned int i=1;i<s-1;++i)
    {
        prior(i-1) += location_probability[i]/3;
        prior(i) += location_probability[i]/3;
        prior(i+1) += location_probability[i]/3;
    }
    if(s!=1)
    {
        //Probability going forward from the last place is split up as:
        // (probability mass)*p_at_new_place     is assigned to the new place
        // (probability mass)*(1-p_at_new_place) is spread evenly over all known places
        // If we had a valency estimate for each known place, then we could improve this prior.
        prior(s-2) += location_probability[s-1]/3;
        prior(s-1) += location_probability[s-1]/3;

        //Spread (1-p_at_new_place)*p evenly over the s known places
        double pmass_to_each_known_place = (1-p_at_new_place)*location_probability[s-1]/((double)3*s);
        for(unsigned int i = 0; i<s;++i)
        {
            prior(i) += pmass_to_each_known_place;    
        }
        //Now, assign (p_at_new_place)*p to the new place
        prior(s) += p_at_new_place*(location_probability[s-1]/3);
    }

    //Assign a small baseline probability to all cells
    //We're generating the prior in a fairly crude way. We don't want it to be too confident.
    prior *= (1-m_dfSmoothingFactor);
    double pmass_teleport = m_dfSmoothingFactor/((double)s+1);
    for(unsigned int i = 0; i<(s+1);++i)
    {
        prior(i) += pmass_teleport;    
    }

    return VNLToSTL(prior);
}

string LeftRightBelowMotionModelPrior::DescribePrior() const
{
    return "Prior: Previous Position + Motion Model\nMotion Model: Left,Right and Below\n";
}

