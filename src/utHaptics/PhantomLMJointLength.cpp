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

#include "PhantomLMJointLength.h"
#include <iostream>
#include <iterator>

#include <utUtil/Logging.h>
#include <utUtil/Exception.h>
#include <utMath/Optimization/GaussNewton.h>
#include <utMath/Optimization/LevenbergMarquardt.h>
#include <utHaptics/Function/PhantomEstJointLength.h>

#include <log4cpp/Category.hh>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomLMJointLength" ) );

namespace ublas = boost::numeric::ublas;

#ifdef HAVE_LAPACK
#include <boost/numeric/bindings/lapack/gesvd.hpp>
namespace lapack = boost::numeric::bindings::lapack;
#endif



namespace Ubitrack { namespace Haptics {

/** internal of PhantomLMCalibration */
#ifdef HAVE_LAPACK


template< typename ForwardIterator1, typename ForwardIterator2 >
Math::Vector< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type, 5  > computePhantomLMJointLengthImp(const ForwardIterator1 iJointAnglesBegin, const ForwardIterator1 iJointAnglesEnd, ForwardIterator2 iPointsBegin,
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l1_est, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l2_est,
							const Math::Vector< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type, 3 > & calib_est
							)
{
	// shortcut to double/float
	typedef typename std::iterator_traits< ForwardIterator1 >::value_type::value_type Type;
	
	unsigned n ( iJointAnglesEnd - iJointAnglesBegin );
	Function::PhantomEstJointLength< Type, ForwardIterator1, ForwardIterator2 > func( iJointAnglesBegin, iJointAnglesEnd, iPointsBegin, l1_est, l2_est, calib_est );
	
	// prepare the measurement vector
	ublas::vector< Type > measurement( func.size() );
	func.buildMeasurementVector( measurement );

	// prepare the input 2-vector to be optimized
	ublas::vector< Type > parameters( func.parameterSize() );
	func.buildParameterVector( parameters );
	
	// perform optimization
	Type residual = Ubitrack::Math::Optimization::levenbergMarquardt( func, parameters, measurement, Math::Optimization::OptTerminate( 200, 1e-6 ), Math::Optimization::OptNoNormalize() );
	LOG4CPP_DEBUG( logger, "Phantom Joint Lengths Estimation result (residual): " << double(residual)
		<< std::endl << "l1: " << parameters(0) << " l2: " << parameters(1)
	);	
	// maybe provide some info about the quality ?
	//if(pResidual)
	//	*pResidual = (double)residual;
	
	// assemble result as a matrix for now -- maybe this should be a different format .. but that would require new datatypes (e.g. Vector< 12 , Type >)
	Math::Vector< Type, 5 > result;
	result( 0 ) = parameters( 0 ); // l1
	result( 1 ) = parameters( 1 ); // l2
	result( 2 ) = parameters( 2 ); // calibx
	result( 3 ) = parameters( 3 ); // caliby
	result( 4 ) = parameters( 4 ); // calibz

	return result;

}

Math::Vector< float, 5 > computePhantomLMJointLength( const std::vector< Math::Vector< float, 3 > > & jointangles, const std::vector< Math::Vector< float, 3 > > & points,
															const float l1_est, const float l2_est, const Math::Vector< float, 3 > & origin_est )
{
	if ( jointangles.size() != points.size() ) {
		UBITRACK_THROW( "Phantom joint length estimation: size mismatch for input vectors." );
	}
	return computePhantomLMJointLengthImp(jointangles.begin(), jointangles.end(), points.begin(), l1_est, l2_est, origin_est);
}

Math::Vector< double, 5 > computePhantomLMJointLength( const std::vector< Math::Vector< double, 3 > > & jointangles, const std::vector< Math::Vector< double, 3 > > & points,
															const double l1_est, const double l2_est, const Math::Vector< double, 3 > & origin_est )
{
	if ( jointangles.size() != points.size() ) {
		UBITRACK_THROW( "Phantom joint length estimation: size mismatch for input vectors." );
	}
	return computePhantomLMJointLengthImp(jointangles.begin(), jointangles.end(), points.begin(), l1_est, l2_est, origin_est);
}
#endif

} } // namespace Ubitrack::Haptics
