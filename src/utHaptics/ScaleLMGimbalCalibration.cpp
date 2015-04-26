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
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.ScaleLMGimbalCalibration" ) );
static log4cpp::Category& optLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.ScaleLMGimbalCalibration.LM" ) );
#include <utMath/Optimization/LevenbergMarquardt.h>

#include <utUtil/Logging.h>
#include <utUtil/Exception.h>
#include <utMath/Optimization/GaussNewton.h>

#include "ScaleLMGimbalCalibration.h"
#include <utHaptics/Function/ScaleFWKOrientationError.h>



namespace ublas = boost::numeric::ublas;

#ifdef HAVE_LAPACK
#include <boost/numeric/bindings/lapack/gesvd.hpp>
namespace lapack = boost::numeric::bindings::lapack;
#endif




namespace Ubitrack { namespace Haptics {

/** internal of ScaleLMGimbalCalibration */
#ifdef HAVE_LAPACK

template< typename ForwardIterator1, typename ForwardIterator2, typename ForwardIterator3, typename ForwardIterator4 >
Math::Matrix< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type , 3, 3 >
    computeScaleLMGimbalCalibrationImp(const ForwardIterator1 iPlatformSensorsBegin, 
										 const ForwardIterator1 iPlatformSensorsEnd,
                                         const ForwardIterator2 iJointAnglesBegin, 
										 const ForwardIterator3 iGimbalAnglesBegin,
                                         const ForwardIterator4 iZRefBegin,
                                         const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l1,
                                         const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l2,
                                         const Math::Matrix< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type, 3 , 3 > angle_correction,
										 const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type optimizationStepSize, 
										 const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type optimizationStepFactor)
{
	// shortcut to double/float
	typedef typename std::iterator_traits< ForwardIterator1 >::value_type::value_type Type;
	
	Function::ScaleFWKOrientationError< Type, ForwardIterator1, ForwardIterator2, ForwardIterator3, ForwardIterator4 > func( iPlatformSensorsBegin, iPlatformSensorsEnd, iJointAnglesBegin, iGimbalAnglesBegin, iZRefBegin, l1, l2, angle_correction );
	
	// prepare the measurement vector
	ublas::vector< Type > measurement( func.size() );
	func.buildMeasurementVector( measurement );

	// prepare the input 6-vector to be optimized
	ublas::vector< Type > parameters( func.parameterSize() );
	func.buildParameterVector( parameters );
	
	// perform optimization
	Type residual = Ubitrack::Math::Optimization::levenbergMarquardt( func, parameters, measurement, Math::Optimization::OptTerminate( 1000, 1e-9 ), Math::Optimization::OptNoNormalize(), 
		Math::Optimization::lmUseSVD, optimizationStepSize, optimizationStepFactor );
	LOG4CPP_INFO( logger, "ScaleGimbalCalibration Optimization result (residual): " << double(residual)
		<< std::endl << "O4 factor: " << parameters(0) << " offset: " << parameters(2)
		<< std::endl << "O5 factor: " << parameters(1) << " offset: " << parameters(3)
		//<< std::endl << "O6 factor: " << parameters(2) << " offset: " << parameters(5)
	);	
	// maybe provide some info about the quality ?
	//if(pResidual)
	//	*pResidual = (double)residual;

	Math::Matrix< Type, 3, 3> cf;
	cf( 0 , 0 ) = 0.0; // j4
	cf( 0 , 1 ) = parameters( 0 ); // k4
	cf( 0 , 2 ) = parameters( 2 ); // m4
	cf( 1 , 0 ) = 0.0; // j5
	cf( 1 , 1 ) = parameters( 1 ); // k5
	cf( 1 , 2 ) = parameters( 3 ); // m5
	cf( 2 , 0 ) = 0.0; // j06
	cf( 2 , 1 ) = 1.0; // k06
	cf( 2 , 2 ) = 0.0; // m06

	return cf;

}


Math::Matrix< float, 3, 3 > computeScaleLMGimbalCalibration( const std::vector< Math::Vector< float, 3 > > & platformsensors,
                                                              const std::vector< Math::Vector< float, 3 > > & jointangles,
                                                              const std::vector< Math::Vector< float, 3 > > & gimbalangles,
                                                              const std::vector< Math::Vector< float, 3 > > & zref,
                                                              const float l1, 
															  const float l2, 
															  const Math::Matrix< float, 3 , 3 > & angle_correction,
															  const float optimizationStepSize, const float optimizationStepFactor)
{
	if (( platformsensors.size() != zref.size()) || ( jointangles.size() != zref.size()) || (gimbalangles.size() != zref.size()) ) {
		UBITRACK_THROW( "Scale workspace calibration: size mismatch for input vectors." );
	}
	return computeScaleLMGimbalCalibrationImp(platformsensors.begin(), platformsensors.end(), jointangles.begin(), gimbalangles.begin(), zref.begin(), l1, l2, angle_correction, optimizationStepSize, optimizationStepFactor);
}

Math::Matrix< double, 3, 3 > computeScaleLMGimbalCalibration( const std::vector< Math::Vector< double, 3 > > & platformsensors,
                                                              const std::vector< Math::Vector< double, 3 > > & jointangles,
                                                              const std::vector< Math::Vector< double, 3 > > & gimbalangles,
                                                              const std::vector< Math::Vector< double, 3 > > & zref,
                                                              const double l1, 
															  const double l2, 
															  const Math::Matrix< double, 3 , 3 > & angle_correction,
															  const double optimizationStepSize, const double optimizationStepFactor)
{
    if (( platformsensors.size() != zref.size()) || ( jointangles.size() != zref.size()) || (gimbalangles.size() != zref.size()) ) {
        UBITRACK_THROW( "Scale workspace calibration: size mismatch for input vectors." );
    }
    return computeScaleLMGimbalCalibrationImp(platformsensors.begin(), platformsensors.end(), jointangles.begin(), gimbalangles.begin(), zref.begin(), l1, l2, angle_correction, optimizationStepSize, optimizationStepFactor);
}

#endif

} } // namespace Ubitrack::Haptics
