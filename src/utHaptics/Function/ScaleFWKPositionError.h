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

#ifndef __UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKPOSITIONERROR_H_INCLUDED__
#define __UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKPOSITIONERROR_H_INCLUDED__

#include <utHaptics.h>
//#include <utMath/Functors/Vector3Functors.h>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

namespace Ubitrack { namespace Haptics { namespace Function {

/**
 * Function that projects a computes a position using forward kinematics and returs the squared distance to the measured position.
 *
 */
template< class VType, typename ForwardIterator1, typename ForwardIterator2, typename ForwardIterator3 >
class ScaleFWKPositionError
{
public:
	/** 
	 * constructor.
	 */
	ScaleFWKPositionError( ForwardIterator1 iPlatformSensorsBegin, ForwardIterator1 iPlatformSensorsEnd, 
		ForwardIterator2 iJointAnglesBegin, ForwardIterator3 iPointsBegin, 
		VType l1, VType l2 )
		: m_iPlatformSensorsBegin( iPlatformSensorsBegin )
		, m_iPlatformSensorsEnd( iPlatformSensorsEnd )
        , m_iJointAnglesBegin( iJointAnglesBegin )
		, m_iPointsBegin(iPointsBegin)
		, m_l1(l1)
		, m_l2(l1)
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
		return 6;
	}


	/**
	 * @param result N-vector to store the result in
	 * @param input containing the parameters ( k1, k2, k3, m1, m2, m3 )
	 */
	template< class VT1, class VT2 > 
	void evaluate( VT1& result, const VT2& input ) const
	{
		namespace ublas = boost::numeric::ublas;
		unsigned i( 0 );

		const VType k1 = input( 0 );
		const VType k2 = input( 1 );
		const VType k3 = input( 2 );
		const VType m1 = input( 3 );
		const VType m2 = input( 4 );
		const VType m3 = input( 5 );

		const VType l1 = m_l1;
		const VType l2 = m_l2;

		ForwardIterator2 itj(m_iJointAnglesBegin);
		ForwardIterator3 iPoints(m_iPointsBegin);
		for (ForwardIterator1 it(m_iPlatformSensorsBegin); it != m_iPlatformSensorsEnd; ++i, ++it, ++itj, ++iPoints) 
		{
			const VType S1 = (*it)( 0 );
			const VType S2 = (*it)( 1 );
			const VType S3 = (*it)( 2 );

			const VType O1 = (*itj)( 0 );
			const VType O2 = (*itj)( 1 );
			const VType O3 = (*itj)( 2 );

			const VType refx = (*iPoints)( 0 );
			const VType refy = (*iPoints)( 1 );
			const VType refz = (*iPoints)( 2 );

			// store the squarred distance from reference to calculated pos
			result(i) = pow(-S1 - l1*cos(O1*k1 + m1)*cos(O2*k2 + m2) - l2*cos(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refx, 2) + 
						pow(-S2 - l1*sin(O1*k1 + m1)*cos(O2*k2 + m2) - l2*sin(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refy, 2) + 
						pow(-S3 + l1*sin(O2*k2 + m2) + l2*sin(O2*k2 + O3*k3 + m2 + m3) + refz, 2);
		}
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
		const VType k1 = input( 0 );
		const VType k2 = input( 1 );
		const VType k3 = input( 2 );
		const VType m1 = input( 3 );
		const VType m2 = input( 4 );
		const VType m3 = input( 5 );

		const VType l1 = m_l1;
		const VType l2 = m_l2;

		unsigned i( 0 );
		ForwardIterator2 itj(m_iJointAnglesBegin);
		ForwardIterator3 iPoints(m_iPointsBegin);
		for (ForwardIterator1 it(m_iPlatformSensorsBegin); it != m_iPlatformSensorsEnd; ++i, ++it, ++itj, ++iPoints) 
		{
			const VType S1 = (*it)( 0 );
			const VType S2 = (*it)( 1 );
			const VType S3 = (*it)( 2 );

			const VType O1 = (*itj)( 0 );
			const VType O2 = (*itj)( 1 );
			const VType O3 = (*itj)( 2 );

			const VType refx = (*iPoints)( 0 );
			const VType refy = (*iPoints)( 1 );
			const VType refz = (*iPoints)( 2 );

			J( i, 0) = (2*O1*l1*sin(O1*k1 + m1)*cos(O2*k2 + m2) + 2*O1*l2*sin(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3))*(-S1 - l1*cos(O1*k1 + m1)*cos(O2*k2 + m2) - l2*cos(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refx) + (-2*O1*l1*cos(O1*k1 + m1)*cos(O2*k2 + m2) - 2*O1*l2*cos(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3))*(-S2 - l1*sin(O1*k1 + m1)*cos(O2*k2 + m2) - l2*sin(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refy);
			J( i, 1) = (2*O2*l1*cos(O2*k2 + m2) + 2*O2*l2*cos(O2*k2 + O3*k3 + m2 + m3))*(-S3 + l1*sin(O2*k2 + m2) + l2*sin(O2*k2 + O3*k3 + m2 + m3) + refz) + (2*O2*l1*sin(O1*k1 + m1)*sin(O2*k2 + m2) + 2*O2*l2*sin(O1*k1 + m1)*sin(O2*k2 + O3*k3 + m2 + m3))*(-S2 - l1*sin(O1*k1 + m1)*cos(O2*k2 + m2) - l2*sin(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refy) + (2*O2*l1*sin(O2*k2 + m2)*cos(O1*k1 + m1) + 2*O2*l2*sin(O2*k2 + O3*k3 + m2 + m3)*cos(O1*k1 + m1))*(-S1 - l1*cos(O1*k1 + m1)*cos(O2*k2 + m2) - l2*cos(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refx);
			J( i, 2) = 2*O3*l2*(-S1 - l1*cos(O1*k1 + m1)*cos(O2*k2 + m2) - l2*cos(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refx)*sin(O2*k2 + O3*k3 + m2 + m3)*cos(O1*k1 + m1) + 2*O3*l2*(-S2 - l1*sin(O1*k1 + m1)*cos(O2*k2 + m2) - l2*sin(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refy)*sin(O1*k1 + m1)*sin(O2*k2 + O3*k3 + m2 + m3) + 2*O3*l2*(-S3 + l1*sin(O2*k2 + m2) + l2*sin(O2*k2 + O3*k3 + m2 + m3) + refz)*cos(O2*k2 + O3*k3 + m2 + m3);
			J( i, 3) = (2*l1*sin(O1*k1 + m1)*cos(O2*k2 + m2) + 2*l2*sin(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3))*(-S1 - l1*cos(O1*k1 + m1)*cos(O2*k2 + m2) - l2*cos(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refx) + (-2*l1*cos(O1*k1 + m1)*cos(O2*k2 + m2) - 2*l2*cos(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3))*(-S2 - l1*sin(O1*k1 + m1)*cos(O2*k2 + m2) - l2*sin(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refy);
			J( i, 4) = (2*l1*cos(O2*k2 + m2) + 2*l2*cos(O2*k2 + O3*k3 + m2 + m3))*(-S3 + l1*sin(O2*k2 + m2) + l2*sin(O2*k2 + O3*k3 + m2 + m3) + refz) + (2*l1*sin(O1*k1 + m1)*sin(O2*k2 + m2) + 2*l2*sin(O1*k1 + m1)*sin(O2*k2 + O3*k3 + m2 + m3))*(-S2 - l1*sin(O1*k1 + m1)*cos(O2*k2 + m2) - l2*sin(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refy) + (2*l1*sin(O2*k2 + m2)*cos(O1*k1 + m1) + 2*l2*sin(O2*k2 + O3*k3 + m2 + m3)*cos(O1*k1 + m1))*(-S1 - l1*cos(O1*k1 + m1)*cos(O2*k2 + m2) - l2*cos(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refx);
			J( i, 5) = 2*l2*(-S1 - l1*cos(O1*k1 + m1)*cos(O2*k2 + m2) - l2*cos(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refx)*sin(O2*k2 + O3*k3 + m2 + m3)*cos(O1*k1 + m1) + 2*l2*(-S2 - l1*sin(O1*k1 + m1)*cos(O2*k2 + m2) - l2*sin(O1*k1 + m1)*cos(O2*k2 + O3*k3 + m2 + m3) + refy)*sin(O1*k1 + m1)*sin(O2*k2 + O3*k3 + m2 + m3) + 2*l2*(-S3 + l1*sin(O2*k2 + m2) + l2*sin(O2*k2 + O3*k3 + m2 + m3) + refz)*cos(O2*k2 + O3*k3 + m2 + m3);
		}
	}

	/** creates a parameter vector based on the initial guess (no correction needed) */
	template< class VT >
	void buildParameterVector( VT& v )
	{
		// set defaults for correction factors and offsets
		// k1, k2, k3, m1, m2, m3
		v( 0 ) = 1.0;
		v( 1 ) = 1.0;
		v( 2 ) = 1.0;
		v( 3 ) = 0.0;
		v( 4 ) = 0.0;
		v( 5 ) = 0.0;
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
	const ForwardIterator1 m_iJointAnglesBegin;
	const ForwardIterator2 m_iPointsBegin;
	const VType m_l1;
	const VType m_l2;
};


} } } // namespace Ubitrack::Haptics::Function

#endif //__UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKINEMATIC_H_INCLUDED__
