/*
 * SurfTools.h
 *
 *  Created on: Oct 27, 2011
 *      Author: Ben Davis
 *      This is an interface which ties Panomatic surf to FabMap.
 */

#ifndef SURFTOOLS_H_
#define SURFTOOLS_H_

#include "KeyPointDetector.h"
#include "KeyPointDescriptor.h"
#include "Image.h"
#include "CImg.h"
#include "InterestPoint.h"
#include "SurfToolsInterface.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string.h>



using namespace cimg_library;
using namespace SurfInterface;
namespace SurfInterface {
class PanoKeyPointVectInsertor: public libsurf::KeyPointInsertor {
public:
	PanoKeyPointVectInsertor(std::vector<libsurf::KeyPoint>& keyPoints) :
		m_KeyPoints(keyPoints) {
	}
	;
	inline virtual void operator()(const libsurf::KeyPoint &keyPoint) {
		m_KeyPoints.push_back(keyPoint);
	}
private:
	std::vector<libsurf::KeyPoint>& m_KeyPoints;
};

class SurfTools: public SurfInterface::SurfToolsInterface {
public:

	/// @brief compute KeyPoint descriptors
	inline static void computeKeyPointDescriptors(libsurf::Image &intImage,
			std::vector<SurfInterface::InterestPoint> &interestPoints,
			int octaves, int intervals, double threshold, bool extended) {
		
		// Information about features extracted. Comment out for debug.
		//cout << "octs: " << octaves << " intervals/scales: " << intervals
		//		<< " threshold: " << threshold << endl;

		std::vector<libsurf::KeyPoint> keyPoints;
		//clock_t start = clock();
		//Construct detector & descriptor
		libsurf::KeyPointDetector detector;
		detector.setMaxOctaves(octaves);
		detector.setMaxScales(intervals);
		detector.setScoreThreshold(threshold);
		libsurf::KeyPointDescriptor descriptor(intImage, extended);

		//Insertor inserts into keyPoints
		PanoKeyPointVectInsertor insertor(keyPoints);

		//Detect & describe
		detector.detectKeypoints(intImage, insertor);

		// resize our interest point array
		interestPoints.reserve(keyPoints.size());

		for (std::vector<libsurf::KeyPoint>::iterator it = keyPoints.begin(); it
				!= keyPoints.end(); ++it) {

			descriptor.assignOrientation(*it);
			descriptor.makeDescriptor(*it);

			//added to populate our InterestPoint Vector data structure


			InterestPoint point((*it)._x, (*it)._y, (*it)._scale, (*it)._score, (*it)._trace);
			int len = descriptor.getDescriptorLength();

			point.descriptor.reserve(len);
			for (int k = 0; k < len; k++) {
				point.descriptor.push_back((*it)._vec[k]);
			}
			interestPoints.push_back(point);

		}
		//clock_t end = clock();
	//	cout << "took:" << float(end-start)/CLOCKS_PER_SEC << " secs" << endl;

	}

	/// @brief compute keypoint descriptors from image file
	inline static void computeInterestPoints(string &filename, std::vector<
			SurfInterface::InterestPoint> &interestPoints, int octaves,
			int intervals, double threshold, bool extended) {

		//Load it
		clock_t start = clock();
		CImg<double> image = CImg<> (filename.c_str());

		//Copy data into a SURF image object
		//unsigned int width = image.width;
		//	unsigned int height = image.height;
		double **pixels = libsurf::Image::AllocateImage(image.width(),
				image.height());

		//convert image to double, normalize to [0..1]
		// why is this 100? shouldn't it be 1.0?
		static const double norm = 1.0 / 255.0;

		for (unsigned int y = 0; y < image.height(); ++y) {
			for (unsigned int x = 0; x < image.width(); ++x) {
				pixels[y][x] = norm * double(image(x, y));
			}
		}
		libsurf::Image intImage(pixels, image.width(), image.height());
			computeKeyPointDescriptors(intImage, interestPoints, octaves,
				intervals, threshold,extended);
		clock_t end = clock();
	//			cout << "surf took:" << float(end-start)/CLOCKS_PER_SEC << " secs and found " << interestPoints.size() << " points." << endl;

		// clean up image
		libsurf::Image::DeallocateImage(pixels, image.height());

	}

	/*	inline static std::vector<libsurf::KeyPoint> computePanoPoints(
	 string &filename, int octaves, int intervals, double threshold) {
	 std::vector<libsurf::KeyPoint> panoKeyPoints;
	 computePanoPoints(filename, panoKeyPoints, octaves, intervals,
	 threshold);
	 return panoKeyPoints;
	 }
	 */

};
}
#endif /* SURFTOOLS_H_ */
