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

#ifndef __libsurf_keypointdescriptor_h
#define __libsurf_keypointdescriptor_h

#include "Image.h"
#include "KeyPoint.h"

namespace libsurf {

class KeyPointDescriptorContext
{
public:
	KeyPointDescriptorContext(	int iSubRegions, int iVecLen, double iOrientation);
	~KeyPointDescriptorContext();

	int				_subRegions;	// number of square subregions (1 direction)

	double			_sin;			// sinus of orientation
	double			_cos;			// cosinus of orientation

	double***		_cmp;			// descriptor components

	void			placeInIndex(	double iMag1, int iOri1,
									double iMag2, int iOri2, double iUIdx, double iVIdx);

	void			placeInIndex2(	double iMag1, int iOri1, double iUIdx, double iVIdx);

private:
	// disallow stupid things
	KeyPointDescriptorContext();
	KeyPointDescriptorContext(const KeyPointDescriptorContext&);
	KeyPointDescriptorContext& operator=(KeyPointDescriptorContext&) throw();

};

class KeyPointDescriptor
{
public:
	KeyPointDescriptor(Image& iImage, bool iExtended = false);
	void makeDescriptor(KeyPoint& ioKeyPoint) const;
	int getDescriptorLength() const;
	void			assignOrientation(KeyPoint& ioKeyPoint) const;
	
private:
	// disallow stupid things
	KeyPointDescriptor();
	KeyPointDescriptor(const KeyPointDescriptor&);
	KeyPointDescriptor& operator=(KeyPointDescriptor&) throw();
	
	// internals
	void			createDescriptor(KeyPointDescriptorContext& iCtx, KeyPoint& ioKeyPoint) const;
	

	// orig image info
	Image&			_image;
	
	// info about the descriptor
	bool			_extended;			// use surf64 or surf128
	int				_subRegions;		// number of square subregions. default = 4
	int				_vecLen;			// length of the vector. 4 for surf 64, 8 for surf 128
	double			_magFactor;

};

}

#endif //__libsurf_keypointdescriptor_h
