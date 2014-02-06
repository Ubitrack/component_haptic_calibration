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

#include <utUtil/Logging.h>
#include <utUtil/Exception.h>
#include <utMath/Optimization/GaussNewton.h>
#include <utMath/Optimization/LevenbergMarquardt.h>
#include <utHaptics/Function/PhantomFWKOrientationError.h>

#include <log4cpp/Category.hh>
// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomLMGimbalCalibration" ) );


namespace ublas = boost::numeric::ublas;

#ifdef HAVE_LAPACK
#include <boost/numeric/bindings/lapack/gesvd.hpp>
namespace lapack = boost::numeric::bindings::lapack;
#endif




namespace Ubitrack { namespace Haptics {

/** internal of PhantomLMGimbalCalibration */
#ifdef HAVE_LAPACK


template< typename ForwardIterator1, typename ForwardIterator2, typename ForwardIterator3 >
Math::Matrix< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type , 3, 4 >
    computePhantomLMGimbalCalibrationImp(const ForwardIterator1 iJointAnglesBegin, const ForwardIterator1 iJointAnglesEnd,
                                         const ForwardIterator2 iGimbalAnglesBegin,
                                         const ForwardIterator3 iZRefBegin,
                                         const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l1,
                                         const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l2,
                                         const Math::Matrix< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type, 3 , 4 > angle_correction,
                                         const Math::Vector< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type, 3 > calib)
{
	// shortcut to double/float
	typedef typename std::iterator_traits< ForwardIterator1 >::value_type::value_type Type;
	
	Function::PhantomFWKOrientationError< Type, ForwardIterator1, ForwardIterator2, ForwardIterator3 > func( iJointAnglesBegin, iJointAnglesEnd, iGimbalAnglesBegin, iZRefBegin, l1, l2, angle_correction, calib );
	
	// prepare the measurement vector
	ublas::vector< Type > measurement( func.size() );
	func.buildMeasurementVector( measurement );

	// prepare the input 6-vector to be optimized
	ublas::vector< Type > parameters( func.parameterSize() );
	func.buildParameterVector( parameters );
	
	// perform optimization
	Type residual = Ubitrack::Math::Optimization::levenbergMarquardt( func, parameters, measurement, Math::Optimization::OptTerminate( 200, 1e-6 ), Math::Optimization::OptNoNormalize() );
	LOG4CPP_DEBUG( logger, "PhantomGimbalCalibration Optimization result (residual): " << double(residual)
		<< std::endl << "O4 factor: " << parameters(0) << " offset: " << parameters(3)
		<< std::endl << "O5 factor: " << parameters(1) << " offset: " << parameters(4)
		<< std::endl << "O6 factor: " << parameters(2) << " offset: " << parameters(5)
	);	
	// maybe provide some info about the quality ?
	//if(pResidual)
	//	*pResidual = (double)residual;
	
	// assemble result as a matrix for now -- maybe this should be a different format .. but that would require new datatypes (e.g. Vector< 12 , Type >)
	Math::Matrix< Type, 3, 4> cf;
	cf( 0 , 0 ) = angle_correction( 0 , 0 ); // k01
	cf( 0 , 1 ) = angle_correction( 0 , 1 ); // m01
	cf( 0 , 2 ) = angle_correction( 0 , 2 ); // k02
	cf( 0 , 3 ) = angle_correction( 0 , 3 ); // m02
	cf( 1 , 0 ) = angle_correction( 1 , 0 ); // k03
	cf( 1 , 1 ) = angle_correction( 1 , 1 ); // m03
	cf( 1 , 2 ) = parameters( 0 ); // k04
	cf( 1 , 3 ) = parameters( 1 ); // m04
	cf( 2 , 0 ) = parameters( 2 ); // k05
	cf( 2 , 1 ) = parameters( 3 ); // m05
	cf( 2 , 2 ) = parameters( 4 ); // k06
	cf( 2 , 3 ) = parameters( 5 ); // m06

	return cf;

}

Math::Matrix< float, 3, 4 > computePhantomLMGimbalCalibration( const std::vector< Math::Vector< float, 3 > > & jointangles,
                                                              const std::vector< Math::Vector< float, 3 > > & gimbalangles,
                                                              const std::vector< Math::Vector< float, 3 > > & zref,
                                                              const Math::Matrix< float, 3 , 4 > angle_correction,
                                                              const float l1, const float l2, Math::Vector< float, 3 > & calib )
{
	if (( jointangles.size() != zref.size()) || (gimbalangles.size() != zref.size()) ) {
		UBITRACK_THROW( "Phantom workspace calibration: size mismatch for input vectors." );
	}
	return computePhantomLMGimbalCalibrationImp(jointangles.begin(), jointangles.end(), gimbalangles.begin(), zref.begin(), l1, l2, angle_correction, calib);
}

Math::Matrix< double, 3, 4 > computePhantomLMGimbalCalibration( const std::vector< Math::Vector< double, 3 > > & jointangles,
                                                              const std::vector< Math::Vector< double, 3 > > & gimbalangles,
                                                              const std::vector< Math::Vector< double, 3 > > & zref,
                                                              const Math::Matrix< double, 3 , 4 > angle_correction,
                                                              const float l1, const double l2, Math::Vector< double, 3 > & calib )
{
    if (( jointangles.size() != zref.size()) || (gimbalangles.size() != zref.size()) ) {
        UBITRACK_THROW( "Phantom workspace calibration: size mismatch for input vectors." );
    }
    return computePhantomLMGimbalCalibrationImp(jointangles.begin(), jointangles.end(), gimbalangles.begin(), zref.begin(), l1, l2, angle_correction, calib);
}

#endif

} } // namespace Ubitrack::Haptics
