/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the 
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup haptic_algorithms
 * @file
 * Functions for Workspace Calibration of Phantom Haptic Devices
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */ 

#include "PhantomLMCalibration.h"
#include <iostream>
#include <iterator>

#include <utUtil/Exception.h>
#include <utMath/GaussNewton.h>
#include <utHaptics/Function/PhantomFWKinematic.h>


#include <log4cpp/Category.hh>

namespace ublas = boost::numeric::ublas;

#ifdef HAVE_LAPACK
#include <boost/numeric/bindings/lapack/gesvd.hpp>
namespace lapack = boost::numeric::bindings::lapack;
#endif

#define OPTIMIZATION_LOGGING
// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomLMCalibration" ) );
static log4cpp::Category& optLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomLMCalibration.LM" ) );
#include <utMath/LevenbergMarquardt.h>

namespace Ubitrack { namespace Haptics {

/** internal of PhantomLMCalibration */
#ifdef HAVE_LAPACK


template< typename ForwardIterator1, typename ForwardIterator2 >
Math::Matrix< 3, 4, typename std::iterator_traits< ForwardIterator1 >::value_type::value_type  > computePhantomLMCalibrationImp(const ForwardIterator1 iJointAnglesBegin, const ForwardIterator1 iJointAnglesEnd, ForwardIterator2 iPointsBegin, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l1, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l2 ) 
{
	// shortcut to double/float
	typedef typename std::iterator_traits< ForwardIterator1 >::value_type::value_type Type;
	
	unsigned n ( iJointAnglesEnd - iJointAnglesBegin );
	Function::PhantomFWKinematic< Type, ForwardIterator1, ForwardIterator2 > func( iJointAnglesBegin, iJointAnglesEnd, iPointsBegin, l1, l2 );
	
	// prepare the measurement vector
	ublas::vector< Type > measurement( func.size() );
	func.buildMeasurementVector( measurement );

	// prepare the input 6-vector to be optimized
	ublas::vector< Type > parameters( func.parameterSize() );
	func.buildParameterVector( parameters );
	
	// perform optimization
	Type residual = Ubitrack::Math::levenbergMarquardt( func, parameters, measurement, Math::OptTerminate( 200, 1e-6 ), Math::OptNoNormalize() );
	LOG4CPP_DEBUG( logger, "PhantomCalibration Optimization result (residual): " << double(residual)
		<< std::endl << "O1 factor: " << parameters(0) << " offset: " << parameters(1)
		<< std::endl << "O2 factor: " << parameters(2) << " offset: " << parameters(3)
		<< std::endl << "O3 factor: " << parameters(4) << " offset: " << parameters(5)
	);	
	// maybe provide some info about the quality ?
	//if(pResidual)
	//	*pResidual = (double)residual;
	
	// assemble result as a matrix for now -- maybe this should be a different format .. but that would require new datatypes (e.g. Vector< 12 , Type >)
	Math::Matrix< 3, 4, Type> cf;
	cf( 0 , 0 ) = parameters( 0 ); // k01
	cf( 0 , 1 ) = parameters( 3 ); // m01
	cf( 0 , 2 ) = parameters( 1 ); // k02
	cf( 0 , 3 ) = parameters( 4 ); // m02
	cf( 1 , 0 ) = parameters( 2 ); // k03
	cf( 1 , 1 ) = parameters( 5 ); // m03
	cf( 1 , 2 ) = 1.0 ; // k04
	cf( 1 , 3 ) = 0.0 ; // m04
	cf( 2 , 0 ) = 1.0 ; // k05
	cf( 2 , 1 ) = 0.0 ; // m05
	cf( 2 , 2 ) = 1.0 ; // k06
	cf( 2 , 3 ) = 0.0 ; // m06

	return cf;

}

Math::Matrix< 3, 4, float > computePhantomLMCalibration( const std::vector< Math::Vector< 3, float > > & jointangles, const std::vector< Math::Vector< 3, float > > & points, 
															const float l1, const float l2 ) 
{
	if ( jointangles.size() != points.size() ) {
		UBITRACK_THROW( "Phantom workspace calibration: size mismatch for input vectors." );
	}
	return computePhantomLMCalibrationImp(jointangles.begin(), jointangles.end(), points.begin(), l1, l2);
}

Math::Matrix< 3, 4, double > computePhantomLMCalibration( const std::vector< Math::Vector< 3, double > > & jointangles, const std::vector< Math::Vector< 3, double > > & points, 
															const double l1, const double l2 ) 
{
	if ( jointangles.size() != points.size() ) {
		UBITRACK_THROW( "Phantom workspace calibration: size mismatch for input vectors." );
	}
	return computePhantomLMCalibrationImp(jointangles.begin(), jointangles.end(), points.begin(), l1, l2);
}
#endif

} } // namespace Ubitrack::Haptics
