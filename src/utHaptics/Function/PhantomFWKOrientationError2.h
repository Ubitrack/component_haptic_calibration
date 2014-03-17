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

template< class VType, typename ForwardIterator1, typename ForwardIterator2, typename ForwardIterator3 >
class PhantomFWKOrientationError2
{
public:
	/** 
	 * constructor.
	 */
	PhantomFWKOrientationError2( ForwardIterator1 iJointAnglesBegin, ForwardIterator1 iJointAnglesEnd,
                               ForwardIterator2 iGimbalAnglesBegin,
                               ForwardIterator3 iZRefBegin, VType l1, VType l2,
                               const Math::Matrix< VType, 3, 3 > &angle_correction,
                               const Math::Vector< VType, 3 >& calib )
		: m_iJointAnglesBegin( iJointAnglesBegin )
		, m_iJointAnglesEnd( iJointAnglesEnd )
        , m_iGimbalAnglesBegin( iGimbalAnglesBegin )
		, m_iZRefBegin( iZRefBegin )
		, m_l1( l1 )
		, m_l2( l1 )
        , m_angle_correction( angle_correction )
		, m_calib( calib )
	{}

	/**
	 * return the size of the result vector containing the distances between the calculated and the measured points
	 */
	unsigned size() const
	{ 
		return ( m_iJointAnglesEnd - m_iJointAnglesBegin ); 
	}

	/** size of the parameter vector */
	unsigned parameterSize() const
	{ 
		// for now only the factors and offsets for the joint angles are optimized
		return 6;
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

		const VType j1 = m_angle_correction( 0, 0 );
		const VType k1 = m_angle_correction( 0, 1 );
		const VType m1 = m_angle_correction( 0, 2 );
		const VType j2 = m_angle_correction( 1, 0 );
		const VType k2 = m_angle_correction( 1, 1 );
		const VType m2 = m_angle_correction( 1, 2 );
		const VType j3 = m_angle_correction( 2, 0 );
		const VType k3 = m_angle_correction( 2, 1 );
		const VType m3 = m_angle_correction( 2, 2 );

		const VType j4 = input( 0 );
		const VType j5 = input( 1 );
		const VType j6 = 0.0; // disabled
		const VType k4 = input( 2 );
		const VType k5 = input( 3 );
		const VType k6 = 0.0; // disabled in 5DOF
		const VType m4 = input( 4 );
		const VType m5 = input( 5 );
		const VType m6 = 0.0; // disabled in 5DOF

		ForwardIterator2 itg(m_iGimbalAnglesBegin);
		ForwardIterator3 iZRef(m_iZRefBegin);
		for (ForwardIterator1 it(m_iJointAnglesBegin); it != m_iJointAnglesEnd; ++i, ++it, ++itg, ++iZRef)
		{
			const VType O1_ = (*it)( 0 );
			const VType O2_ = (*it)( 1 );
			const VType O3_ = (*it)( 2 );
			const VType O4_ = (*itg)( 0 );
			const VType O5_ = (*itg)( 1 ); 
			const VType O6_ = (*itg)( 2 ); 

			const VType O1 = j1*pow(O1_, 2) + k1*O1_ + m1;
			const VType O2 = j2*pow(O2_, 2) + k2*O2_ + m2;
			const VType O3 = j3*pow(O3_, 2) + k3*O3_ + m3;
			const VType O4 = j4*pow(O4_, 2) + k4*O4_ + m4;
			const VType O5 = j5*pow(O5_, 2) + k5*O5_ + m5;
			const VType O6 = j6*pow(O6_, 2) + k6*O6_ + m6;

			const double sO1 = sin(O1);
			const double cO1 = cos(O1);
			const double sO2 = sin(O2);
			const double cO2 = cos(O2);
			const double sO3 = sin(O3);
			const double cO3 = cos(O3);
			const double sO4 = sin(O4);
			const double cO4 = cos(O4);
			const double sO5 = sin(O5);
			const double cO5 = cos(O5);
			const double sO6 = sin(O6);
			const double cO6 = cos(O6);

			const Math::Vector< VType, 3> rotref = Math::normalize(*iZRef);
			const VType rotrefx = rotref( 0 );
			const VType rotrefy = rotref( 1 );
			const VType rotrefz = rotref( 2 );

            
			// store result as angle difference in sin(angle) + 1 (range: 0-2)
			result(i) = -rotrefx*(cO5*(cO1*sO4 - cO3*cO4*sO1) + sO1*sO3*sO5) 
				- rotrefy*(cO3*sO5 + cO4*cO5*sO3) 
				- rotrefz*(-cO1*sO3*sO5 + cO5*(cO1*cO3*cO4 + sO1*sO4)) + 1;

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

		const VType j1 = m_angle_correction( 0, 0 );
		const VType k1 = m_angle_correction( 0, 1 );
		const VType m1 = m_angle_correction( 0, 2 );
		const VType j2 = m_angle_correction( 1, 0 );
		const VType k2 = m_angle_correction( 1, 1 );
		const VType m2 = m_angle_correction( 1, 2 );
		const VType j3 = m_angle_correction( 2, 0 );
		const VType k3 = m_angle_correction( 2, 1 );
		const VType m3 = m_angle_correction( 2, 2 );

		const VType j4 = input( 0 );
		const VType j5 = input( 1 );
		const VType j6 = 0.0; // disabled
		const VType k4 = input( 2 );
		const VType k5 = input( 3 );
		const VType k6 = 1.0; // disabled in 5DOF
		const VType m4 = input( 4 );
		const VType m5 = input( 5 );
		const VType m6 = 0.0; // disabled in 5DOF
		
		unsigned i( 0 );
		ForwardIterator2 itg(m_iGimbalAnglesBegin);
		ForwardIterator3 iZRef(m_iZRefBegin);
		for (ForwardIterator1 it(m_iJointAnglesBegin); it != m_iJointAnglesEnd; ++i, ++it, ++itg, ++iZRef)
		{

			const VType O1_ = (*it)( 0 );
			const VType O2_ = (*it)( 1 );
			const VType O3_ = (*it)( 2 );
			const VType O4_ = (*itg)( 0 );
			const VType O5_ = (*itg)( 1 ); 
			const VType O6_ = (*itg)( 2 ); 

			const VType O1 = j1*pow(O1_, 2) + k1*O1_ + m1;
			const VType O2 = j2*pow(O2_, 2) + k2*O2_ + m2;
			const VType O3 = j3*pow(O3_, 2) + k3*O3_ + m3;
			const VType O4 = j4*pow(O4_, 2) + k4*O4_ + m4;
			const VType O5 = j5*pow(O5_, 2) + k5*O5_ + m5;
			const VType O6 = j6*pow(O6_, 2) + k6*O6_ + m6;

			const double sO1 = sin(O1);
			const double cO1 = cos(O1);
			const double sO2 = sin(O2);
			const double cO2 = cos(O2);
			const double sO3 = sin(O3);
			const double cO3 = cos(O3);
			const double sO4 = sin(O4);
			const double cO4 = cos(O4);
			const double sO5 = sin(O5);
			const double cO5 = cos(O5);
			const double sO6 = sin(O6);
			const double cO6 = cos(O6);

			const Math::Vector< VType, 3> rotref = Math::normalize(*iZRef);
			const VType rotrefx = rotref( 0 );
			const VType rotrefy = rotref( 1 );
			const VType rotrefz = rotref( 2 );

			J( i, 0 ) = pow(O4, 2)*cO5*rotrefy*sO3*sO4 - cO5*rotrefx*(pow(O4, 2)*cO1*cO4 + pow(O4, 2)*cO3*sO1*sO4) - cO5*rotrefz*(-pow(O4, 2)*cO1*cO3*sO4 + pow(O4, 2)*cO4*sO1);
			J( i, 1 ) = -rotrefx*(pow(O5, 2)*cO5*sO1*sO3 - pow(O5, 2)*sO5*(cO1*sO4 - cO3*cO4*sO1)) - rotrefy*(pow(O5, 2)*cO3*cO5 - pow(O5, 2)*cO4*sO3*sO5) - rotrefz*(-pow(O5, 2)*cO1*cO5*sO3 - pow(O5, 2)*sO5*(cO1*cO3*cO4 + sO1*sO4));
			J( i, 2 ) = O4*cO5*rotrefy*sO3*sO4 - cO5*rotrefx*(O4*cO1*cO4 + O4*cO3*sO1*sO4) - cO5*rotrefz*(-O4*cO1*cO3*sO4 + O4*cO4*sO1);
			J( i, 3 ) = -rotrefx*(O5*cO5*sO1*sO3 - O5*sO5*(cO1*sO4 - cO3*cO4*sO1)) - rotrefy*(O5*cO3*cO5 - O5*cO4*sO3*sO5) - rotrefz*(-O5*cO1*cO5*sO3 - O5*sO5*(cO1*cO3*cO4 + sO1*sO4));
			J( i, 4 ) = -cO5*rotrefx*(cO1*cO4 + cO3*sO1*sO4) + cO5*rotrefy*sO3*sO4 - cO5*rotrefz*(-cO1*cO3*sO4 + cO4*sO1);
			J( i, 5 ) = -rotrefx*(cO5*sO1*sO3 - sO5*(cO1*sO4 - cO3*cO4*sO1)) - rotrefy*(cO3*cO5 - cO4*sO3*sO5) - rotrefz*(-cO1*cO5*sO3 - sO5*(cO1*cO3*cO4 + sO1*sO4));

		}
		OPT_LOG_TRACE( "rphi_jacobian: " << J );
	}

	/** creates a parameter vector based on the initial guess (no correction needed) */
	template< class VT >
	void buildParameterVector( VT& v )
	{
		// set defaults for correction factors and offsets
		// k4, k5, m4, m5
		v( 0 ) = 0.0;
		v( 1 ) = 0.0;
		v( 2 ) = 1.0;
		v( 3 ) = 1.0;
		v( 4 ) = 0.0;
		v( 5 ) = 0.0;
	}

	/** creates a measurement vector based on the un-corrected joint angles and their corresponding reference points */
	template< class VT >
	void buildMeasurementVector( VT& v )
	{
		namespace ublas = boost::numeric::ublas;
		unsigned i = 0;
		for (ForwardIterator1 it(m_iJointAnglesBegin); it != m_iJointAnglesEnd; ++i, ++it) 
		{
			v(i) = 0;
		}
		
	}
	
protected:

	const ForwardIterator1 m_iJointAnglesBegin;
	const ForwardIterator1 m_iJointAnglesEnd;
	const ForwardIterator2 m_iGimbalAnglesBegin;
	const ForwardIterator3 m_iZRefBegin;
	const VType m_l1;
	const VType m_l2;
    const Math::Matrix< VType, 3, 3 > m_angle_correction;
	const Math::Vector< VType, 3 > m_calib;
};

} 

} } // namespace Ubitrack::Haptics::Function

#endif //__UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKINEMATIC_H_INCLUDED__
