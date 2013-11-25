/*
 * SurfTools.h
 *
 *  Created on: Oct 27, 2011
 *      Author: Ben Davis
 *      This is an interface which ties ETHZ surf to FabMap.
 */

#ifndef SURFTOOLS_H_
#define SURFTOOLS_H_

#include "Image.h"
#include "CImg.h"
#include "InterestPoint.h"
#include "SurfToolsInterface.h"

//SURF
#ifdef WIN32
#include "surfWINDLL.h"
#endif
#include "imload.h"
#include "surflib.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string.h>

using namespace cimg_library;
using namespace SurfInterface;
using namespace surf;

namespace SurfInterface {
class SurfTools: public SurfInterface::SurfToolsInterface {
	 public:
	/// @brief compute keypoint descriptors from image file
	inline static void computeInterestPoints(string &filename, std::vector<
			SurfInterface::InterestPoint> &interestPoints, int octaves,
			int intervals, double threshold, bool extended) {

		//Load it
		clock_t start = clock();

		//Load it
		CImg<double> image = CImg<> (filename.c_str());

		//Convert to greyscale
		//The correct way to do this is via
		//const CImg<double> dest = image.get_RGBtoYCbCr().channel(0);
		//Which extracts the luminance channel from YCbCr
		//This resize way is 10x faster, so we prefer it for online usage.
		//However, still to be verified if it impacts recognition performance.
		image.resize(-100, -100, -100, 1);

		//Copy data into a SURF image object
		const double normalizer = 255.00001;
		Image *im = new Image(image.width(), image.height());
		for (unsigned int y = 0; y < image.height(); y++)
			for (unsigned int x = 0; x < image.width(); x++)
				im->setPix(x, y, image(x, y) / normalizer);

		std::vector<Ipoint> keyPoints;

		// fabmap values
		//The rest are probably best left at defaults.
		bool m_SURF_doubleImageSize = false;
		bool m_SURF_upright = true;
		int m_SURF_initLobe = 3;
		int m_SURF_samplingStep = 2;
		int m_SURF_indexSize = intervals;

		//Compute the SURF descriptor
		surfDetDes(im, keyPoints, threshold, m_SURF_doubleImageSize,
				m_SURF_initLobe, intervals, octaves, m_SURF_upright, extended,
				m_SURF_indexSize);


		// copy descriptors and InterestPoint data into our own format
		interestPoints.reserve(keyPoints.size());
		for (std::vector<Ipoint>::iterator it = keyPoints.begin(); it
				!= keyPoints.end(); ++it) {
			//added to populate our InterestPoint Vector data structure
			InterestPoint point((*it).x, (*it).y, (*it).scale, (*it).strength,
					-1);
			int len = 64;

			if (extended) // if big descriptor
				len = 128;

			point.descriptor.reserve(len);
			for (int k = 0; k < len; k++) {
				point.descriptor.push_back((*it).ivec[k]);
			}
			interestPoints.push_back(point);
		}

		//Tidy up
		delete im;

		clock_t end = clock();
		//cout << "eth surf took:" << float(end - start) / CLOCKS_PER_SEC
		//		<< " secs and found " << interestPoints.size() << " points."
		//		<< endl;

	}

};
}
#endif /* SURFTOOLS_H_ */
