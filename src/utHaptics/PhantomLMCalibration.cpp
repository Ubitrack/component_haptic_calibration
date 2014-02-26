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

#include <iostream>
#include <iterator>

#include <log4cpp/Category.hh>

// extensive logging for optimization
#define OPTIMIZATION_LOGGING
// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomLMCalibration" ) );
static log4cpp::Category& optLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomLMCalibration.LM" ) );
#include <utMath/Optimization/LevenbergMarquardt.h>


#include <utUtil/Logging.h>
#include <utUtil/Exception.h>
#include <utMath/Optimization/GaussNewton.h>

#include "PhantomLMCalibration.h"
#include <utHaptics/Function/PhantomFWKPositionError.h>


namespace ublas = boost::numeric::ublas;

#ifdef HAVE_LAPACK
#include <boost/numeric/bindings/lapack/gesvd.hpp>
namespace lapack = boost::numeric::bindings::lapack;
#endif




namespace Ubitrack { namespace Haptics {

/** internal of PhantomLMCalibration */
#ifdef HAVE_LAPACK

// if available use LEVMAR LM Implementation
#ifdef HAVE_LEVMAR

// warning of duplicate definition here ..
#undef HAVE_LAPACK
#include "levmar.h"

template< typename LMFuncType >
void position_error_func(double *p, double *x, int m, int n, void *data) {
	LMFuncType *func = reinterpret_cast< LMFuncType* >(data);
	ublas::vector< double > result( func->size() );
	ublas::vector< double > parameters( func->parameterSize() );

	for (int i=0; i<m; i++) {
		parameters( i ) = p[i];
	}

	func->evaluate(result, parameters);

	for (int i=0; i<n; i++) {
		x[i] = result( i );
	}

}

template< typename LMFuncType >
void position_jacobian_func(double *p, double *jac, int m, int n, void *data) {
	LMFuncType *func = reinterpret_cast< LMFuncType* >(data);
	typedef typename Math::Matrix< double, 0, 0 >::base_type MatType;
	MatType pJacobian( n, m );
	ublas::vector< double > parameters( func->parameterSize() );

	for (int i=0; i<m; i++) {
		parameters( i ) = p[i];
	}

	func->jacobian(parameters, pJacobian);

	unsigned jidx = 0;
	for (int i=0; i<n; i++) {
		for (int j=0; j<m; j++) {
			jac[jidx++] = pJacobian( i, j );
		}
	}

}

template< typename ForwardIterator1, typename ForwardIterator2 >
Math::Matrix< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type , 3, 4 > computePhantomLMCalibrationImp(const ForwardIterator1 iJointAnglesBegin, const ForwardIterator1 iJointAnglesEnd, ForwardIterator2 iPointsBegin, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l1, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l2,
							const Math::Vector< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type, 3 > calib, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type optimizationStepSize, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type optimizationStepFactor
							)
{

	// shortcut to double/float
	typedef typename std::iterator_traits< ForwardIterator1 >::value_type::value_type Type;
	typedef typename Function::PhantomFWKPositionError< Type, ForwardIterator1, ForwardIterator2 > LMFuncType;
	unsigned n ( iJointAnglesEnd - iJointAnglesBegin );
	LMFuncType func( iJointAnglesBegin, iJointAnglesEnd, iPointsBegin, l1, l2, calib );
	
	// prepare the measurement vector
	ublas::vector< Type > measurement( func.size() );
	func.buildMeasurementVector( measurement );

	// prepare the input 6-vector to be optimized
	ublas::vector< Type > parameters( func.parameterSize() );
	func.buildParameterVector( parameters );


	// levmar integration starts here
	int ret;
	unsigned m = func.parameterSize();

	// initialize parameters and measurement vectors (copy for now)
	std::vector< Type > p(m);
	std::vector< Type > x(n);

	for (unsigned i=0; i<m; i++) {
		p[i] = parameters( i );
	}

	for (unsigned i=0; i<n; i++) {
		x[i] = measurement( i );
	}

	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];

	opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
	opts[4]= LM_DIFF_DELTA; // relevant only if the Jacobian is approximated using finite differences; specifies forward differencing 
    //opts[4]=-LM_DIFF_DELTA; // specifies central differencing to approximate Jacobian; more accurate but more expensive to compute!

	ret = dlevmar_der(position_error_func< LMFuncType >, position_jacobian_func< LMFuncType >, p.data(), x.data(), m, n, 1000, opts, info, NULL, NULL, (void *)(&func));


  printf("Levenberg-Marquardt returned %d in %g iter, reason %g\nSolution: ", ret, info[5], info[6]);
  for(int i=0; i<m; ++i)
    printf("%.7g ", p[i]);
  printf("\n\nMinimization info:\n");
  for(int i=0; i<LM_INFO_SZ; ++i)
    printf("%g ", info[i]);
  printf("\n");


	for (unsigned i=0; i<m; i++) {
		parameters( i ) = p[i];
	}

	/**

	// perform optimization
	Type residual = Ubitrack::Math::Optimization::levenbergMarquardt( func, parameters, measurement, Math::Optimization::OptTerminate( 1000, 1e-9 ), Math::Optimization::OptNoNormalize(), 
		Math::Optimization::lmUseSVD, optimizationStepSize, optimizationStepFactor );
	// maybe provide some info about the quality ?
	//if(pResidual)
	//	*pResidual = (double)residual;


	**/

	double residual = 0.0;
	LOG4CPP_INFO( logger, "PhantomCalibration Optimization result (residual): " << double(residual)
		<< std::endl << "O1 factor: " << parameters(0) << " offset: " << parameters(3)
		<< std::endl << "O2 factor: " << parameters(1) << " offset: " << parameters(4)
		<< std::endl << "O3 factor: " << parameters(2) << " offset: " << parameters(5)
	);	

	// assemble result as a matrix for now -- maybe this should be a different format .. but that would require new datatypes (e.g. Vector< 12 , Type >)
	Math::Matrix< Type, 3, 4> cf;
	cf( 0 , 0 ) = parameters( 0 ); // k1
	cf( 0 , 1 ) = parameters( 3 ); // m1
	cf( 0 , 2 ) = parameters( 1 ); // k2
	cf( 0 , 3 ) = parameters( 4 ); // m2
	cf( 1 , 0 ) = parameters( 2 ); // k3
	cf( 1 , 1 ) = parameters( 5 ); // m3
	cf( 1 , 2 ) = 1.0 ; // k4
	cf( 1 , 3 ) = 0.0 ; // m4
	cf( 2 , 0 ) = 1.0 ; // k5
	cf( 2 , 1 ) = 0.0 ; // m5
	cf( 2 , 2 ) = 1.0 ; // k6
	cf( 2 , 3 ) = 0.0 ; // m6

	return cf;

}

// only version for double exists atm

Math::Matrix< double, 3, 4 > computePhantomLMCalibration( const std::vector< Math::Vector< double, 3 > > & jointangles, const std::vector< Math::Vector< double, 3 > > & points, 
															const double l1, const double l2, Math::Vector< double, 3 > & calib, const double optimizationStepSize, const double optimizationStepFactor )
{
	if ( jointangles.size() != points.size() ) {
		UBITRACK_THROW( "Phantom workspace calibration: size mismatch for input vectors." );
	}
	return computePhantomLMCalibrationImp(jointangles.begin(), jointangles.end(), points.begin(), l1, l2, calib, optimizationStepSize, optimizationStepFactor);
}


#else // HAVE_LEVMAR

template< typename ForwardIterator1, typename ForwardIterator2 >
Math::Matrix< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type , 3, 4 > computePhantomLMCalibrationImp(const ForwardIterator1 iJointAnglesBegin, const ForwardIterator1 iJointAnglesEnd, ForwardIterator2 iPointsBegin, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l1, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type l2,
							const Math::Vector< typename std::iterator_traits< ForwardIterator1 >::value_type::value_type, 3 > calib, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type optimizationStepSize, 
							const typename std::iterator_traits< ForwardIterator1 >::value_type::value_type optimizationStepFactor
							)
{
	// shortcut to double/float
	typedef typename std::iterator_traits< ForwardIterator1 >::value_type::value_type Type;
	
	unsigned n ( iJointAnglesEnd - iJointAnglesBegin );
	Function::PhantomFWKPositionError< Type, ForwardIterator1, ForwardIterator2 > func( iJointAnglesBegin, iJointAnglesEnd, iPointsBegin, l1, l2, calib );
	
	// prepare the measurement vector
	ublas::vector< Type > measurement( func.size() );
	func.buildMeasurementVector( measurement );

	// prepare the input 6-vector to be optimized
	ublas::vector< Type > parameters( func.parameterSize() );
	func.buildParameterVector( parameters );
	
	// perform optimization
	Type residual = Ubitrack::Math::Optimization::levenbergMarquardt( func, parameters, measurement, Math::Optimization::OptTerminate( 1000, 1e-9 ), Math::Optimization::OptNoNormalize(), 
		Math::Optimization::lmUseSVD, optimizationStepSize, optimizationStepFactor );
	LOG4CPP_INFO( logger, "PhantomCalibration Optimization result (residual): " << double(residual)
		<< std::endl << "O1 factor: " << parameters(0) << " offset: " << parameters(3)
		<< std::endl << "O2 factor: " << parameters(1) << " offset: " << parameters(4)
		<< std::endl << "O3 factor: " << parameters(2) << " offset: " << parameters(5)
	);	
	// maybe provide some info about the quality ?
	//if(pResidual)
	//	*pResidual = (double)residual;
	
	// assemble result as a matrix for now -- maybe this should be a different format .. but that would require new datatypes (e.g. Vector< 12 , Type >)
	Math::Matrix< Type, 3, 4> cf;
	cf( 0 , 0 ) = parameters( 0 ); // k1
	cf( 0 , 1 ) = parameters( 3 ); // m1
	cf( 0 , 2 ) = parameters( 1 ); // k2
	cf( 0 , 3 ) = parameters( 4 ); // m2
	cf( 1 , 0 ) = parameters( 2 ); // k3
	cf( 1 , 1 ) = parameters( 5 ); // m3
	cf( 1 , 2 ) = 1.0 ; // k4
	cf( 1 , 3 ) = 0.0 ; // m4
	cf( 2 , 0 ) = 1.0 ; // k5
	cf( 2 , 1 ) = 0.0 ; // m5
	cf( 2 , 2 ) = 1.0 ; // k6
	cf( 2 , 3 ) = 0.0 ; // m6

	return cf;

}

Math::Matrix< float, 3, 4 > computePhantomLMCalibration( const std::vector< Math::Vector< float, 3 > > & jointangles, const std::vector< Math::Vector< float, 3 > > & points, 
															const float l1, const float l2, Math::Vector< float, 3 > & calib, const float optimizationStepSize, const float optimizationStepFactor )
{
	if ( jointangles.size() != points.size() ) {
		UBITRACK_THROW( "Phantom workspace calibration: size mismatch for input vectors." );
	}
	return computePhantomLMCalibrationImp(jointangles.begin(), jointangles.end(), points.begin(), l1, l2, calib, optimizationStepSize, optimizationStepFactor);
}

Math::Matrix< double, 3, 4 > computePhantomLMCalibration( const std::vector< Math::Vector< double, 3 > > & jointangles, const std::vector< Math::Vector< double, 3 > > & points, 
															const double l1, const double l2, Math::Vector< double, 3 > & calib, const double optimizationStepSize, const double optimizationStepFactor )
{
	if ( jointangles.size() != points.size() ) {
		UBITRACK_THROW( "Phantom workspace calibration: size mismatch for input vectors." );
	}
	return computePhantomLMCalibrationImp(jointangles.begin(), jointangles.end(), points.begin(), l1, l2, calib, optimizationStepSize, optimizationStepFactor);
}

#endif // HAVE_LEVMAR


#endif

} } // namespace Ubitrack::Haptics
