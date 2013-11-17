/*
 * InterestPoint.h
 *
 *  Created on: Oct 28, 2011
 *      Author: Ben Davis
 */

#ifndef INTERESTPOINT_H_
#define INTERESTPOINT_H_


/*
* The class is defined as a common interface for SURF algorithm output.
* X,Y,Scale,Descriptor being the common denominators between implementations.
* Sometimes fudging has to go on in the adapter class, however, we abstract this. Individual implementations can do the necessary compatibility fudging. There will always be knobs and dials to twiddle.
*
*/
#include <vector>
#include <ostream>

namespace SurfInterface {

class InterestPoint
{
	public:
		InterestPoint();
		InterestPoint ( double _x, double _y, double _scale);
		InterestPoint ( double _x, double _y, double _scale, std::vector<double> _descriptor);
		InterestPoint ( double _x, double _y, double _scale, double _strength, int _trace );

		double		x, y;
		double		scale;
		double		strength; // blob response...
		int			trace;
		double		ori;

		std::vector<double> descriptor;
};

inline InterestPoint::InterestPoint()
{

}

inline InterestPoint::InterestPoint ( double _x, double _y, double _scale) :
		x ( _x ), y ( _y ), scale ( _scale ), strength ( 0.0 ), trace ( 0 ), descriptor ( 0 )
{

}
inline InterestPoint::InterestPoint ( double _x, double _y, double _scale, std::vector<double> _descriptor) :
		x ( _x ), y ( _y ), scale ( _scale ), strength ( 0.0 ), trace ( 0 ), descriptor ( _descriptor )
{

}

inline InterestPoint::InterestPoint ( double _x, double _y, double _scale, double _strength, int _trace ) :
		x ( _x ), y ( _y ), scale ( _scale ), strength ( _strength ), trace ( _trace ), descriptor ( 0 )
{

}

inline bool operator < ( const InterestPoint & iA, const InterestPoint & iB )
{
	return ( iA.strength < iB.strength );
}

inline std::ostream& operator<< ( std::ostream &out, const InterestPoint &keyPoint )
{
	// Similar to SURF's main.cpp:
	double sc = 2.5 * keyPoint.scale;
	sc *= sc; // circular regions with diameter 5 x scale
	out << keyPoint.x			// x-location of the interest point
	<< " " << keyPoint.y 	// y-location of the interest point
	<< " " << 1.0 / sc 		// 1/r^2
	<< " " << keyPoint.strength 	// This doesn't necessarily get reported by all SURF implementations. 	//(*ipts)[n]->strength /* 0.0 */
	<< " " << 1.0 / sc 		// 1/r^2
	<< " " << keyPoint.trace;

	for ( unsigned int descriptor_index = 0; descriptor_index < keyPoint.descriptor.size(); ++descriptor_index )
	{
		out << " " << keyPoint.descriptor[descriptor_index];
	}

	return out;
}

}
#endif /* INTERESTPOINT_H_ */

