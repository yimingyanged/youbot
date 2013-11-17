/*
* Copyright (C) 2007-2008 Anael Orlinski
*
* This file is part of Panomatic.
*
* Panomatic is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* 
* Panomatic is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with Panomatic; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <iostream>

#include "KeyPoint.h"
#include "KeyPointDescriptor.h"
#include "MathStuff.h"
#include "WaveFilter.h"
#include <cmath>
#include <map>



using namespace libsurf;
using namespace std;

LUT<0, 83> Exp1(exp, 0.5, -0.08);
LUT<0, 40> Exp2(exp, 0.5, -0.125);

KeyPointDescriptor::KeyPointDescriptor(Image& iImage, bool iExtended) : 
	_image(iImage), _extended(iExtended)
{
	// init some parameters
	_subRegions = 4;

	_vecLen = 4;
	if (_extended)
		_vecLen = 8;
	
	_magFactor = 12.0 / _subRegions;

}

void KeyPointDescriptor::makeDescriptor(KeyPoint& ioKeyPoint) const
{
	// create a descriptor context
	KeyPointDescriptorContext aCtx(_subRegions, _vecLen, ioKeyPoint._ori);

	// create the storage in the keypoint
	ioKeyPoint.allocVector(getDescriptorLength());

	// assign the orientation
	//assignOrientation(ioKeyPoint);

	// create a vector
	createDescriptor(aCtx, ioKeyPoint);

	// transform back to vector

	// fill the vector with the values of the square...
	// remember the shift of 1 to drop outborders.
	size_t aV = 0;
	for(int aYIt = 1; aYIt < _subRegions+1; ++aYIt)
	{
		for(int aXIt = 1; aXIt < _subRegions+1; ++aXIt)
		{
			for(int aVIt = 0; aVIt < _vecLen; ++aVIt)
			{
				double a = aCtx._cmp[aYIt][aXIt][aVIt];
				ioKeyPoint._vec[aV] = a;
				aV++;
			}
		}
	}


	// normalize
	Math::Normalize(ioKeyPoint._vec, getDescriptorLength());

}

void KeyPointDescriptor::assignOrientation(KeyPoint& ioKeyPoint) const
{
	unsigned int aRX = Math::Round(ioKeyPoint._x);
	unsigned int aRY = Math::Round(ioKeyPoint._y);
	int aStep = (int)(ioKeyPoint._scale + 0.8);
	
	WaveFilter aWaveFilter(2.0 * ioKeyPoint._scale + 1.6, _image);

	multimap<double, double> aRespVector;
	
	// compute haar wavelet responses in a circular neighborhood of 6s
	for (int aYIt = -9; aYIt <= 9; aYIt++)
	{ 
		int aSY = aRY + aYIt * aStep;
		for (int aXIt = -9; aXIt <= 9; aXIt++)
		{
			int aSX = aRX + aXIt * aStep;

			// keep points in a circular region of diameter 6s
			unsigned int aSqDist = aXIt * aXIt + aYIt * aYIt;
			if (aSqDist <= 81 && aWaveFilter.checkBounds(aSX, aSY))
			{
				double aWavX = aWaveFilter.getWx(aSX, aSY);
				double aWavY = aWaveFilter.getWy(aSX, aSY);
				double aWavResp = sqrt(aWavX * aWavX + aWavY * aWavY);
				if (aWavResp > 0)
				{
					double aAngle = atan2(aWavY, aWavX);
					aRespVector.insert(make_pair(aAngle,				aWavResp * Exp1(aSqDist)));
					aRespVector.insert(make_pair(aAngle + 2*PI,			aWavResp * Exp1(aSqDist)));
				}
			}
		}
	}
	
	//estimate orientation using a sliding window of PI/3
	double aMeanOrientation = 0;
	double aArcPonderation = aRespVector.begin()->second;
	double aMax = 0;
	double aPonderedDirection = aRespVector.begin()->first * aArcPonderation;
	
	multimap<double, double>::iterator aWindowBegin, aWindowEnd;
	aWindowBegin = aWindowEnd = aRespVector.begin();
	
	while (aWindowBegin->first < PI && aWindowEnd != aRespVector.end())
	{
		if (aWindowEnd->first - aWindowBegin->first < PI/3)
        {
            if (aMax < aArcPonderation)
            {
				aMeanOrientation = aPonderedDirection / aArcPonderation;
				aMax = aArcPonderation;              
            }
            aWindowEnd++;
            aArcPonderation += aWindowEnd->second;
            aPonderedDirection += aWindowEnd->second * aWindowEnd->first;
        }
        else
        {
	        aArcPonderation -= aWindowBegin->second;          
	        aPonderedDirection -= aWindowBegin->second * aWindowBegin->first;
	        aWindowBegin++;
	    }
	}

	ioKeyPoint._ori = aMeanOrientation;

}

void KeyPointDescriptor::createDescriptor(KeyPointDescriptorContext& iCtx, KeyPoint& ioKeyPoint) const
{
	// create the vector of features by analyzing a square patch around the point.
	// for this the current patch (x,y) will be translated in rotated coordinates (u,v)

	double aX = ioKeyPoint._x;
	double aY = ioKeyPoint._y;
	double aS = ioKeyPoint._scale * 1.65; // multiply by this nice constant

	// make integer values from double ones
	int aIntX = Math::Round(aX);
	int aIntY = Math::Round(aY);
	int aIntS = Math::Round(aS / 2.0);	
	if (aIntS < 1) aIntS = 1;

	// calc subpixel shift
	double aSubX = aX - aIntX;
	double aSubY = aY - aIntY;

	// calc subpixel shift in rotated coordinates
	double aSubV = iCtx._cos * aSubY + iCtx._sin * aSubX;
	double aSubU = - iCtx._sin * aSubY + iCtx._cos * aSubX;

	// calc step of sampling
	double aStepSample = aS * _magFactor;

	// make a bounding box around the rotated patch square. 
	double aRadius = (1.414 * aStepSample) * (_subRegions + 1) / 2.0;
	int aIntRadius = Math::Round(aRadius / aIntS);
	if (aS < 1) aS = 1;

	// go through all the pixels in the bounding box
	for (int aYIt = -aIntRadius; aYIt <= aIntRadius; ++aYIt)
	{
		for (int aXIt = -aIntRadius; aXIt <= aIntRadius; ++aXIt)
		{
			// calculate U,V rotated values from X,Y taking in account subpixel correction
			// divide by step sample to put in index units
			double aU = ((( - iCtx._sin * aYIt + iCtx._cos * aXIt) * aIntS) - aSubU) / aStepSample;
			double aV = (((iCtx._cos * aYIt + iCtx._sin * aXIt) * aIntS) - aSubV) / aStepSample;

			// compute location of sample in terms of real array coordinates 
			double aUIdx = _subRegions / 2.0 - 0.5 + aU;
			double aVIdx = _subRegions / 2.0 - 0.5 + aV;

			// test if some bins will be filled.
			if (aUIdx >= -1.0 && aUIdx < _subRegions &&
				aVIdx >= -1.0 && aVIdx < _subRegions )
			{
				int aXSample = aIntS * aXIt + aIntX;
				int aYSample = aIntS * aYIt + aIntY;
				int aStep = (int)aS;

				WaveFilter aWaveFilter(aStep, _image);
				if (!aWaveFilter.checkBounds(aXSample, aYSample))
					continue;

				double aExp = Exp2((int)(aV * aV + aU * aU)); 

				double aWavX = aWaveFilter.getWx(aXSample, aYSample) * aExp;
				double aWavY = aWaveFilter.getWy(aXSample, aYSample) * aExp;

				double aWavXR = (iCtx._cos * aWavX + iCtx._sin * aWavY);
				double aWavYR = (iCtx._sin * aWavX - iCtx._cos * aWavY);

				// due to the rotation, the patch has to be dispatched in 2 bins in each direction
				// get the bins and weight for each of them
				// shift by 1 to avoid checking bounds
				const int aBin1U = (int)(aUIdx + 1.0);
				const int aBin2U = aBin1U + 1;
				const int aBin1V = (int)(aVIdx + 1.0);
				const int aBin2V = aBin1V + 1;

				const double aWeightBin1U = aBin1U - aUIdx;
				const double aWeightBin2U = 1 - aWeightBin1U;

				const double aWeightBin1V = aBin1V - aVIdx;
				const double aWeightBin2V = 1 - aWeightBin1V;

				if(_extended) 
				{
					int aBin = (aWavYR <= 0) ? 0 : 1;
					iCtx._cmp[aBin1V][aBin1U][aBin] += aWeightBin1V * aWeightBin1U * aWavXR;
					iCtx._cmp[aBin2V][aBin1U][aBin] += aWeightBin2V * aWeightBin1U * aWavXR;
					iCtx._cmp[aBin1V][aBin2U][aBin] += aWeightBin1V * aWeightBin2U * aWavXR;
					iCtx._cmp[aBin2V][aBin2U][aBin] += aWeightBin2V * aWeightBin2U * aWavXR;

					aBin += 2;
					double aVal = fabs(aWavXR);
					iCtx._cmp[aBin1V][aBin1U][aBin] += aWeightBin1V * aWeightBin1U * aVal;
					iCtx._cmp[aBin2V][aBin1U][aBin] += aWeightBin2V * aWeightBin1U * aVal;
					iCtx._cmp[aBin1V][aBin2U][aBin] += aWeightBin1V * aWeightBin2U * aVal;
					iCtx._cmp[aBin2V][aBin2U][aBin] += aWeightBin2V * aWeightBin2U * aVal;

					aBin = (aWavXR <= 0) ? 4 : 5;
					iCtx._cmp[aBin1V][aBin1U][aBin] += aWeightBin1V * aWeightBin1U * aWavYR;
					iCtx._cmp[aBin2V][aBin1U][aBin] += aWeightBin2V * aWeightBin1U * aWavYR;
					iCtx._cmp[aBin1V][aBin2U][aBin] += aWeightBin1V * aWeightBin2U * aWavYR;
					iCtx._cmp[aBin2V][aBin2U][aBin] += aWeightBin2V * aWeightBin2U * aWavYR;

					aBin += 2;
					aVal = fabs(aWavYR);
					iCtx._cmp[aBin1V][aBin1U][aBin] += aWeightBin1V * aWeightBin1U * aVal;
					iCtx._cmp[aBin2V][aBin1U][aBin] += aWeightBin2V * aWeightBin1U * aVal;
					iCtx._cmp[aBin1V][aBin2U][aBin] += aWeightBin1V * aWeightBin2U * aVal;
					iCtx._cmp[aBin2V][aBin2U][aBin] += aWeightBin2V * aWeightBin2U * aVal;

				} 
				else 
				{
					int aBin = (aWavXR <= 0) ? 0 : 1;
					iCtx._cmp[aBin1V][aBin1U][aBin] += aWeightBin1V * aWeightBin1U * aWavXR;
					iCtx._cmp[aBin2V][aBin1U][aBin] += aWeightBin2V * aWeightBin1U * aWavXR;
					iCtx._cmp[aBin1V][aBin2U][aBin] += aWeightBin1V * aWeightBin2U * aWavXR;
					iCtx._cmp[aBin2V][aBin2U][aBin] += aWeightBin2V * aWeightBin2U * aWavXR;

					aBin = (aWavYR <= 0) ? 2 : 3;
					iCtx._cmp[aBin1V][aBin1U][aBin] += aWeightBin1V * aWeightBin1U * aWavYR;
					iCtx._cmp[aBin2V][aBin1U][aBin] += aWeightBin2V * aWeightBin1U * aWavYR;
					iCtx._cmp[aBin1V][aBin2U][aBin] += aWeightBin1V * aWeightBin2U * aWavYR;
					iCtx._cmp[aBin2V][aBin2U][aBin] += aWeightBin2V * aWeightBin2U * aWavYR;

				}
			}		
		}
	}
}


KeyPointDescriptorContext::KeyPointDescriptorContext(	int iSubRegions, 
														int iVecLen, 
														double iOrientation) :
	_subRegions(iSubRegions), _sin(sin(iOrientation)), _cos(cos(iOrientation))
{
	// allocate and initialize the components of the vector
	// the idea is to allocate 2 more in each direction and
	// shift access by 1 to discard out of bounds.

	int aExtSub = _subRegions + 2;
	_cmp = new double **[aExtSub];
	for (int aYIt = 0; aYIt < aExtSub; ++aYIt)
	{
		_cmp[aYIt] = new double *[aExtSub];
		for (int aXIt = 0; aXIt < aExtSub; ++aXIt)
		{
			_cmp[aYIt][aXIt] = new double[iVecLen];
			for (int aVIt = 0; aVIt < iVecLen; ++aVIt)
				_cmp[aYIt][aXIt][aVIt] = 0;
		}
	}
}


KeyPointDescriptorContext::~KeyPointDescriptorContext()
{
	// destroy the components of the vector
	int aExtSub = _subRegions + 2;
	for (int aYIt = 0; aYIt < aExtSub; ++aYIt)
	{
		for (int aXIt = 0; aXIt < aExtSub; ++aXIt)
		{
			delete[] _cmp[aYIt][aXIt];
		}
		delete[] _cmp[aYIt];
	}
}


int KeyPointDescriptor::getDescriptorLength() const
{
	return _vecLen * _subRegions * _subRegions;
}
