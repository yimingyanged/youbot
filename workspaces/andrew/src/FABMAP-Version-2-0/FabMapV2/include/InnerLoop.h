#ifndef FABMAP_V2_INNER_LOOP_H_XGSJLHAKK7478
#define FABMAP_V2_INNER_LOOP_H_XGSJLHAKK7478 1

#include "FabMap.h"

//Calculates p(e=1 | L) for given word, for a place for which we have exactly one observation, in which the word was in state z.
//If we have a constant detector model, this will have only two possible values (corresponding to z=0 and z=1)
//The model supports only binary observations (z in {0,1})
//though it shouldn't be too hard to take some account of integer-value z,
//for example by considering the probability of multiple detector failures.
inline double FabMapCalculator::GetP_e_1_given_z(const unsigned int nWordID,
                                                 const unsigned int state_z) const
{
    //                           p(z=1|e=1)p(e=1)
    //p(e=1|z=1) =     ------------------------------------
    //                  p(z=1|e=1)p(e=1) + p(z=1|e=0)p(e=0)
    //
    //
    //                               p(e=1)
    //           =     ---------------------------------------
    //                  p(e=1)   + p(z=1|e=0)(1-p(e=1))/p(z=1|e=1)
    //
    //
    //
    //                           p(z=0|e=1)p(e=1)
    //p(e=1|z=0) =     ------------------------------------
    //                  p(z=0|e=1)p(e=1) + p(z=0|e=0)p(e=0)
    //
    //
    //                               p(e=1)
    //           =     ---------------------------------------
    //                  p(e=1)   + p(z=0|e=0)(1-p(e=1))/p(z=0|e=1)
    //
    //We don't know p(e), so we use p(z) as a prior.

    const double p_e = Marginals[nWordID];

    if(state_z == 0)
    {
        //p(e=1|z=0)
        return p_e/(p_e + (1.0-p_e)*(1.0 - p_z_1_given_e_0[nWordID])/(1.0 - p_z_1_given_e_1[nWordID]));
    }
    else
    {
        //p(e=1|z=1)
        return p_e/(p_e + (1.0-p_e)*p_z_1_given_e_0[nWordID]/p_z_1_given_e_1[nWordID]);
    }

}

inline double FabMapCalculator::GetP_z_Given_Zp_and_E(const unsigned int z_id,
                                                      const unsigned int state_z,
                                                      const unsigned int state_e,
                                                      const unsigned int state_zp) const
{
    //Returns the probability p(za = state_za | ea = state_e, zp = state_sp)
    //where zp is the the parent of za (i.e. z_id) in the CL tree
    double prob;

    //Get detector term
    //detector_term_alpha = p(z = !state_z | e = state_e)
    //detector_term_beta = p(z = state_z | e = state_e)

    //Assume state_z = 1
    double detector_term_alpha;
    if(state_e > 0)
    {   //p(z = !state_z | e = 1) = p(z = 0 | e = 1)
        detector_term_alpha = 1.0 - p_z_1_given_e_1[z_id];
    }
    else
    {   //p(z = !state_z | e = 0) = p(z = 0 | e = 0)
        detector_term_alpha = 1.0 - p_z_1_given_e_0[z_id];
    }
    const double detector_term_beta  = 1.0 - detector_term_alpha;


    //p(z = state_z)p(z = !state_z | e = state_e)p(z = !state_z | zp = state_zp)
    //Assume state_z =1
    if(state_zp == 0)
    {
        //p(z = 1)p(z = 0 | e = state_e)p(z = 0 | zp = 0)
        const double alpha =      Marginals[z_id]*detector_term_alpha*RelevantNotConditionals(z_id);
        const double beta = (1.0-Marginals[z_id])*detector_term_beta*(1.0 - RelevantNotConditionals(z_id));
        prob = (1.0/(1.0 + alpha/beta));
    }
    else
    {
        //p(z = 1)p(z = 0 | e = state_e)p(z = 0 | zp = 1)
        const double alpha =      Marginals[z_id]*detector_term_alpha*(1.0 -  RelevantConditionals(z_id));
        const double beta = (1.0-Marginals[z_id])*detector_term_beta*RelevantConditionals(z_id);
        prob = (1.0/(1.0 + alpha/beta));
    }

    //Now, if necessary state_z was not one, change to 1-prob 
    if(state_z == 0)
    {
        return (1.0 - prob);
    }
    else
    {
        return prob;
    }
}

// Calculates p(z_i | z_pi, L).
inline double FabMapCalculator::GetP_z_Given_Zp(const unsigned int z_i,
                                                const unsigned int state_z,
                                                const unsigned int state_zp,
                                                const PlaceModel &word_exists_prob) const
{
    //Returns the probability p(z_i = state_zi | zp = state_zp, L)
    //where zp is the the parent of z_i in the CL tree
    //and the location L is defined by the word_exists_prob vector
    double prob;

    //Assume state_z = 1
    if(state_zp == 0)
    {
        prob = (word_exists_prob[z_i]/(1.0 + K1[z_i]*RelevantNotConditionals(z_i)/(1.0 - RelevantNotConditionals(z_i))));
    }
    else
    {
        prob = (word_exists_prob[z_i]/(1.0 + K1[z_i]*(1.0 -  RelevantConditionals(z_i))/RelevantConditionals(z_i)));
    }

    //Now, if necessary state_z was not one, change to 1-prob 
    if(state_z == 0)
    {
        return (1.0 - prob);
    }
    else
    {
        return prob;
    }
}

#endif //FABMAP_V2_INNER_LOOP_H_XGSJLHAKK7478

