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
 * @ingroup haptics
 * @file
 * objective function for workspace calibration using levenberg-marquardt fittint
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */

#ifndef __UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKORIENTATIONERROR_H_INCLUDED__
#define __UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKORIENTATIONERROR_H_INCLUDED__

#include <utHaptics.h>
#include <utMath/VectorFunctions.h>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

namespace Ubitrack { namespace Haptics { namespace Function {

/**
 * Function that projects a computes a position using forward kinematics and returs the squared distance to the measured position.
 *
 */

template< class VType, typename ForwardIterator1, typename ForwardIterator2, typename ForwardIterator3, typename ForwardIterator4 >
class ScaleFWKOrientationError
{
public:
	/** 
	 * constructor.
	 */
	ScaleFWKOrientationError( ForwardIterator1 iPlatformSensorsBegin, ForwardIterator1 iPlatformSensorsEnd,
							   ForwardIterator2 iJointAnglesBegin, 
                               ForwardIterator3 iGimbalAnglesBegin,
                               ForwardIterator4 iZRefBegin, VType l1, VType l2,
                               const Math::Matrix< VType, 3, 3 > &angle_correction)
		: m_iPlatformSensorsBegin( iPlatformSensorsBegin )
		, m_iPlatformSensorsEnd( iPlatformSensorsEnd )
        , m_iJointAnglesBegin( iJointAnglesBegin )
		, m_iGimbalAnglesBegin( iGimbalAnglesBegin )
		, m_iZRefBegin( iZRefBegin )
		, m_l1( l1 )
		, m_l2( l1 )
        , m_angle_correction( angle_correction )
	{}

	/**
	 * return the size of the result vector containing the distances between the calculated and the measured points
	 */
	unsigned size() const
	{ 
		return ( m_iPlatformSensorsEnd - m_iPlatformSensorsBegin ); 
	}

	/** size of the parameter vector */
	unsigned parameterSize() const
	{ 
		// for now only the factors and offsets for the joint angles are optimized
		return 4;
	}


	/**
	 * @param result N-vector to store the result in
	 * @param input containing the parameters ( k04, k05, k06, m04, m05, m06 )
	 */
	template< class VT1, class VT2 > 
	void evaluate( VT1& result, const VT2& input ) const
	{
		namespace ublas = boost::numeric::ublas;
		unsigned i( 0 );

		const VType k1 = m_angle_correction( 0, 1 );
		const VType m1 = m_angle_correction( 0, 2 );
		const VType k2 = m_angle_correction( 1, 1 );
		const VType m2 = m_angle_correction( 1, 2 );
		const VType k3 = m_angle_correction( 2, 1 );
		const VType m3 = m_angle_correction( 2, 2 );

		const VType k4 = input( 0 );
		const VType k5 = input( 1 );
		const VType k6 = 0.0; // disabled in 5DOF
		const VType m4 = input( 2 );
		const VType m5 = input( 3 );
		const VType m6 = 0.0; // disabled in 5DOF

		ForwardIterator2 itj(m_iJointAnglesBegin);
		ForwardIterator3 itg(m_iGimbalAnglesBegin);
		ForwardIterator4 iZRef(m_iZRefBegin);
		for (ForwardIterator1 it(m_iPlatformSensorsBegin); it != m_iPlatformSensorsEnd; ++i, ++it, ++itj, ++itg, ++iZRef)
		{
			const VType S1 = (*it)( 0 );
			const VType S2 = (*it)( 1 );
			const VType S3 = (*it)( 2 );

			const VType O1 = (*itj)( 0 );
			const VType O2 = (*itj)( 1 );
			const VType O3 = (*itj)( 2 );
			const VType O4 = (*itg)( 0 );
			const VType O5 = (*itg)( 1 );
			const VType rotrefx = (*iZRef)( 0 );
			const VType rotrefy = (*iZRef)( 1 );
			const VType rotrefz = (*iZRef)( 2 );
            
			// store result as angle difference in sin(angle) + 1 (range: 0-2)
			result(i) = -rotrefx*((sin(O5*k5 + m5)*cos(O2*k2 + O3*k3 + m2 + m3) + sin(O2*k2 + O3*k3 + m2 + m3)*cos(O4*k4 + m4)*cos(O5*k5 + m5))*cos(O1*k1 + m1) + sin(O1*k1 + m1)*sin(O4*k4 + m4)*cos(O5*k5 + m5)) - 
						rotrefy*((sin(O5*k5 + m5)*cos(O2*k2 + O3*k3 + m2 + m3) + sin(O2*k2 + O3*k3 + m2 + m3)*cos(O4*k4 + m4)*cos(O5*k5 + m5))*sin(O1*k1 + m1) - sin(O4*k4 + m4)*cos(O1*k1 + m1)*cos(O5*k5 + m5)) - 
						rotrefz*(-sin(O5*k5 + m5)*sin(O2*k2 + O3*k3 + m2 + m3) + cos(O4*k4 + m4)*cos(O5*k5 + m5)*cos(O2*k2 + O3*k3 + m2 + m3)) + 1;

		}
		OPT_LOG_TRACE( "rphi_results: " << result );
	}
	
	/**
	 * @param result vector to store the result in
	 * @param input containing the parameters (to be optimized)
	 * @param J matrix to store the jacobian (evaluated for input) in
	 */
	template< class VT1, class VT2, class MT > 
	void evaluateWithJacobian( VT1& result, const VT2& input, MT& J ) const
	{
		// TODO: implement as one function (more efficient)
		evaluate( result, input );
		jacobian( input, J );
	}

	/**
	 * @param input containing the parameters (to be optimized)
	 * @param J matrix to store the jacobian (evaluated for input) in
	 */
	template< class VT2, class MT > 
	void jacobian( const VT2& input, MT& J ) const
	{

		const VType k1 = m_angle_correction( 0, 1 );
		const VType m1 = m_angle_correction( 0, 2 );
		const VType k2 = m_angle_correction( 1, 1 );
		const VType m2 = m_angle_correction( 1, 2 );
		const VType k3 = m_angle_correction( 2, 1 );
		const VType m3 = m_angle_correction( 2, 2 );

		const VType k4 = input( 0 );
		const VType k5 = input( 1 );
		const VType k6 = 0.0; // disabled in 5DOF
		const VType m4 = input( 2 );
		const VType m5 = input( 3 );
		const VType m6 = 0.0; // disabled in 5DOF
		
		unsigned i( 0 );
		ForwardIterator2 itj(m_iJointAnglesBegin);
		ForwardIterator3 itg(m_iGimbalAnglesBegin);
		ForwardIterator4 iZRef(m_iZRefBegin);
		for (ForwardIterator1 it(m_iPlatformSensorsBegin); it != m_iPlatformSensorsEnd; ++i, ++it, ++itj, ++itg, ++iZRef)
		{

			const Math::Vector< VType, 3> rotref = Math::normalize(*iZRef);

			const VType S1 = (*it)( 0 );
			const VType S2 = (*it)( 1 );
			const VType S3 = (*it)( 2 );

			const VType O1 = (*itj)( 0 );
			const VType O2 = (*itj)( 1 );
			const VType O3 = (*itj)( 2 );
			const VType O4 = (*itg)( 0 );
			const VType O5 = (*itg)( 1 ); 
			const VType O6 = (*itg)( 2 ); 
			const VType rotrefx = rotref( 0 );
			const VType rotrefy = rotref( 1 );
			const VType rotrefz = rotref( 2 );

			J( i, 0 ) = O4*rotrefy*sin(O3*k3 + m3)*sin(O4*k4 + m4)*cos(O5*k5 + m5) - rotrefx*(O4*sin(O1*k1 + m1)*sin(O4*k4 + m4)*cos(O3*k3 + m3) + O4*cos(O1*k1 + m1)*cos(O4*k4 + m4))*cos(O5*k5 + m5) - rotrefz*(O4*sin(O1*k1 + m1)*cos(O4*k4 + m4) - O4*sin(O4*k4 + m4)*cos(O1*k1 + m1)*cos(O3*k3 + m3))*cos(O5*k5 + m5);
			J( i, 1 ) = -rotrefx*(-O5*(-sin(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4) + sin(O4*k4 + m4)*cos(O1*k1 + m1))*sin(O5*k5 + m5) + O5*sin(O1*k1 + m1)*sin(O3*k3 + m3)*cos(O5*k5 + m5)) - rotrefy*(-O5*sin(O3*k3 + m3)*sin(O5*k5 + m5)*cos(O4*k4 + m4) + O5*cos(O3*k3 + m3)*cos(O5*k5 + m5)) - rotrefz*(-O5*(sin(O1*k1 + m1)*sin(O4*k4 + m4) + cos(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4))*sin(O5*k5 + m5) - O5*sin(O3*k3 + m3)*cos(O1*k1 + m1)*cos(O5*k5 + m5));
			J( i, 2 ) = -rotrefx*(sin(O1*k1 + m1)*sin(O4*k4 + m4)*cos(O3*k3 + m3) + cos(O1*k1 + m1)*cos(O4*k4 + m4))*cos(O5*k5 + m5) + rotrefy*sin(O3*k3 + m3)*sin(O4*k4 + m4)*cos(O5*k5 + m5) - rotrefz*(sin(O1*k1 + m1)*cos(O4*k4 + m4) - sin(O4*k4 + m4)*cos(O1*k1 + m1)*cos(O3*k3 + m3))*cos(O5*k5 + m5);
			J( i, 3 ) = -rotrefx*(-(-sin(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4) + sin(O4*k4 + m4)*cos(O1*k1 + m1))*sin(O5*k5 + m5) + sin(O1*k1 + m1)*sin(O3*k3 + m3)*cos(O5*k5 + m5)) - rotrefy*(-sin(O3*k3 + m3)*sin(O5*k5 + m5)*cos(O4*k4 + m4) + cos(O3*k3 + m3)*cos(O5*k5 + m5)) - rotrefz*(-(sin(O1*k1 + m1)*sin(O4*k4 + m4) + cos(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4))*sin(O5*k5 + m5) - sin(O3*k3 + m3)*cos(O1*k1 + m1)*cos(O5*k5 + m5));

		}
		OPT_LOG_TRACE( "rphi_jacobian: " << J );
	}

	/** creates a parameter vector based on the initial guess (no correction needed) */
	template< class VT >
	void buildParameterVector( VT& v )
	{
		// set defaults for correction factors and offsets
		// k4, k5, m4, m5
		v( 0 ) = 1.0;
		v( 1 ) = 1.0;
		v( 2 ) = 0.0;
		v( 3 ) = 0.0;
	}

	/** creates a measurement vector based on the un-corrected joint angles and their corresponding reference points */
	template< class VT >
	void buildMeasurementVector( VT& v )
	{
		namespace ublas = boost::numeric::ublas;
		unsigned i = 0;
		for (ForwardIterator1 it(m_iPlatformSensorsBegin); it != m_iPlatformSensorsEnd; ++i, ++it) 
		{
			v(i) = 0;
		}
		
	}
	
protected:

	const ForwardIterator1 m_iPlatformSensorsBegin;
	const ForwardIterator1 m_iPlatformSensorsEnd;
	const ForwardIterator2 m_iJointAnglesBegin;
	const ForwardIterator3 m_iGimbalAnglesBegin;
	const ForwardIterator4 m_iZRefBegin;
	const VType m_l1;
	const VType m_l2;
    const Math::Matrix< VType, 3, 3 > m_angle_correction;
};

} 

} } // namespace Ubitrack::Haptics::Function

#endif //__UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKINEMATIC_H_INCLUDED__
