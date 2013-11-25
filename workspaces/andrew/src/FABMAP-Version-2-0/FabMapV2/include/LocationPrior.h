#ifndef LOCATIONPRIOR_H
#define LOCATIONPRIOR_H 1

#include "TypeDefinitions.h"
#include <vnl/vnl_vector.h>
#include <vector>
#include <string>

// Abstract base class for prior providers.
class LocationPriorProvider
{
public:
    LocationPriorProvider(double prob_NewPlace);
    virtual ~LocationPriorProvider() {}

    virtual std::vector<double> GetLocationPrior(const LocationProbabilityContainer &location_probability) const =0;
    virtual std::string DescribePrior() const = 0;
    std::vector<double> ExcludeLocalMatches(const std::vector<double> &existing_prior,
                                            unsigned int k) const;
protected:
    static std::vector<double> VNLToSTL(const vnl_vector<double> &v);
    double p_at_new_place; //The prior probability that a topological link with an unknown endpoint leads to a new place.
};

// This class returns a uniform prior over the set of existing places
class UniformLocationPrior : public LocationPriorProvider
{
public:
    UniformLocationPrior(double prob_NewPlace);
    ~UniformLocationPrior() {};

    std::vector<double> GetLocationPrior(const LocationProbabilityContainer &location_probability) const;
    std::string DescribePrior() const;
};

// This class takes the previous position estimate, and produces a prior by transforming this position estimate
// through a motion model. The motion model is that the robot randomly moves to one of the adjacent places.
// Currently the topology is assumed to be a chain and you can specify a bias in the forward or backward direction.
class LeftRightMotionModelPrior : public LocationPriorProvider
{
public:
    LeftRightMotionModelPrior(double prob_NewPlace);
    LeftRightMotionModelPrior(double prob_NewPlace, double motion_bias);
    ~LeftRightMotionModelPrior() {};

    std::vector<double> GetLocationPrior(const LocationProbabilityContainer &location_probability) const;
    std::string DescribePrior() const;
private:
    // Directional bias. 0.5 for equal probability.
    const double m_dfBias;
    // By default we build a substantial uniform uncertainty into the position prior
    // (because or motion model is extremely crude, so we don't want to give it much weight).
    const double m_dfSmoothingFactor;
};

// This class takes the previous position estimate, and produces a prior by transforming this position estimate
// through a motion model.
// The motion model is that the robot randomly moves to one of the adjacent places, or remains where it is.
// Currently the topology is assumed to be a chain.
class LeftRightBelowMotionModelPrior : public LocationPriorProvider
{
public:
    LeftRightBelowMotionModelPrior(double prob_NewPlace);
    ~LeftRightBelowMotionModelPrior() {};

    std::vector<double> GetLocationPrior(const LocationProbabilityContainer &location_probability) const;
    std::string DescribePrior() const;
private:
    const double m_dfSmoothingFactor;
};

#endif //LOCATIONPRIOR_H
