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
 * Functions for Workspace Calibration of Scale Haptic Devices
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */ 

#include <iostream>
#include <iterator>

#include <log4cpp/Category.hh>

// extensive logging for optimization
#define OPTIMIZATION_LOGGING
// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.ScaleLMCalibration" ) );
static log4cpp::Category& optLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.ScaleLMCalibration.LM" ) );
#include <utMath/Optimization/LevenbergMarquardt.h>


#include <utUtil/Logging.h>
#include <utUtil/Exception.h>
#include <utMath/Optimization/GaussNewton.h>

#include "ScaleLMJointCalibration.h"
#include <utHaptics/Function/ScaleFWKPositionError.h>


namespace ublas = boost::numeric::ublas;

#ifdef HAVE_LAPACK
#include <boost/numeric/bindings/lapack/gesvd.hpp>
namespace lapack = boost::numeric::bindings::lapack;
#endif




namespace Ubitrack { namespace Haptics {

/** internal of ScaleLMCalibration */
#ifdef HAVE_LAPACK

template< typename ForwardIterator1, typename ForwardIterator2, typename ForwardIterator3 >
Math::Matrix< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type , 3, 3 > computeScaleLMCalibrationImp(const ForwardIterator1 iPlatformSensorsBegin, const ForwardIterator1 iPlatformSensorsEnd, const ForwardIterator2 iJointAnglesBegin, ForwardIterator3 iPointsBegin, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l1, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l2,
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type optimizationStepSize, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type optimizationStepFactor
							)
{
	// shortcut to double/float
	typedef typename std::iterator_traits< ForwardIterator1 >::value_type::value_type Type;
	
	unsigned n ( iPlatformSensorsBegin - iPlatformSensorsEnd );
	Function::ScaleFWKPositionError< Type, ForwardIterator1, ForwardIterator2 , ForwardIterator3 > func( iPlatformSensorsBegin, iPlatformSensorsEnd, iJointAnglesBegin, iPointsBegin, l1, l2);
	
	// prepare the measurement vector
	ublas::vector< Type > measurement( func.size() );
	func.buildMeasurementVector( measurement );

	// prepare the input 6-vector to be optimized
	ublas::vector< Type > parameters( func.parameterSize() );
	func.buildParameterVector( parameters );
	
	// perform optimization
	Type residual = Ubitrack::Math::Optimization::levenbergMarquardt( func, parameters, measurement, Math::Optimization::OptTerminate( 1000, 1e-9 ), Math::Optimization::OptNoNormalize(), 
		Math::Optimization::lmUseCholesky, optimizationStepSize, optimizationStepFactor );
	LOG4CPP_DEBUG( logger, "ScaleCalibration Optimization result (residual): " << double(residual)
		<< std::endl << "O1 factor: " << parameters(0) << " offset: " << parameters(3)
		<< std::endl << "O2 factor: " << parameters(1) << " offset: " << parameters(4)
		<< std::endl << "O3 factor: " << parameters(2) << " offset: " << parameters(5)
	);	
	// maybe provide some info about the quality ?
	//if(pResidual)
	//	*pResidual = (double)residual;
	
	// assemble result as a matrix for now -- maybe this should be a different format .. but that would require new datatypes (e.g. Vector< 12 , Type >)
	Math::Matrix< Type, 3, 3> cf;

	cf( 0 , 0 ) = 0.0; // j1
	cf( 0 , 1 ) = parameters( 0 ); // k1
	cf( 0 , 2 ) = parameters( 3 ); // m1
	cf( 1 , 0 ) = 0.0; // j2
	cf( 1 , 1 ) = parameters( 1 ); // k2
	cf( 1 , 2 ) = parameters( 4 ); // m2
	cf( 2 , 0 ) = 0.0; // j3
	cf( 2 , 1 ) = parameters( 2 ); // k3
	cf( 2 , 2 ) = parameters( 5 ); // m3

	return cf;

}

Math::Matrix< float, 3, 3 > computeScaleLMCalibration( const std::vector< Math::Vector< float, 3 > > & platformsensors, const std::vector< Math::Vector< float, 3 > > & jointangles, const std::vector< Math::Vector< float, 3 > > & points, 
															const float l1, const float l2, const float optimizationStepSize, const float optimizationStepFactor )
{
	if (( platformsensors.size() != points.size()) || ( jointangles.size() != points.size() )) {
		UBITRACK_THROW( "Scale workspace calibration: size mismatch for input vectors." );
	}
	return computeScaleLMCalibrationImp(platformsensors.begin(), platformsensors.end(), jointangles.begin(), points.begin(), l1, l2, optimizationStepSize, optimizationStepFactor);
}

Math::Matrix< double, 3, 3 > computeScaleLMCalibration( const std::vector< Math::Vector< double, 3 > > & platformsensors, const std::vector< Math::Vector< double, 3 > > & jointangles, const std::vector< Math::Vector< double, 3 > > & points, 
															const double l1, const double l2, const double optimizationStepSize, const double optimizationStepFactor )
{
	if (( platformsensors.size() != points.size()) || ( jointangles.size() != points.size() )) {
		UBITRACK_THROW( "Scale workspace calibration: size mismatch for input vectors." );
	}
	return computeScaleLMCalibrationImp(platformsensors.begin(), platformsensors.end(), jointangles.begin(), points.begin(), l1, l2, optimizationStepSize, optimizationStepFactor);
}

#endif

} } // namespace Ubitrack::Haptics
