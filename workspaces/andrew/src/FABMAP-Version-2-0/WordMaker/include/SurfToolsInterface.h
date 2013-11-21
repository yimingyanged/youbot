/*
 * SurfToolsInterface.h
 *
 *  Created on: Oct 27, 2011
 *      Author: Ben Davis
 *      This is an interface which ties any surf library to FabMap.
 */

#ifndef SURFTOOLSINTERFACE_H_
#define SURFTOOLSINTERFACE_H_

#include "InterestPoint.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string.h>

namespace SurfInterface {

class SurfToolsInterface {
public:
	/// @brief compute keypoint descriptors from image file
	static void computeInterestPoints(string &filename, std::vector<
			SurfInterface::InterestPoint> &interestPoints, int octaves,
			int intervals, double threshold, bool extended);
};
}
#endif /* SURFTOOLS_H_ */
