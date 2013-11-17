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

#ifndef __libsurf_image_h
#define __libsurf_image_h

namespace libsurf {

// forward declaration
class IntegralImage;

class Image
{
public:
	Image() : _ii(0), _width(0), _height(0) {};
	
	// Constructor from a pixel array
	Image(double **iPixels, unsigned int iWidth, unsigned int iHeight);

	// setup the integral image
	void init(double **iPixels, unsigned int iWidth, unsigned int iHeight);

	// cleanup
	void clean();

	// Destructor
	~Image();

	// Accessors
	inline double **		getPixels()			{ return _pixels; }
	inline double **		getIntegralImage()	{ return _ii; }
	inline unsigned int		getWidth()			{ return _width; }
	inline unsigned int		getHeight()			{ return _height; }

	// allocate and deallocate integral image pixels
	static double **		AllocateImage(unsigned int iWidth, unsigned int iHeight);
	static void				DeallocateImage(double **iImagePtr, unsigned int iHeight);

private:
	
	// prepare the integral image
	void					buildIntegralImage();
	
	// pixel data of the image
	double**				_pixels;
	
	// image size
	unsigned int			_width;
	unsigned int			_height;

	// integral image	
	double**				_ii; // Data of the integral image Like data[lines][rows]
	
};	

}

#endif //__libsurf_image_h

