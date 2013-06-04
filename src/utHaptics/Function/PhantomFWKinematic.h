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

#ifndef __UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKINEMATIC_H_INCLUDED__
#define __UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKINEMATIC_H_INCLUDED__
 
#include <utMath/Functors/Vector3Functors.h>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

namespace Ubitrack { namespace Haptics { namespace Function {

/**
 * Function that projects a computes a position using forward kinematics and returs the squared distance to the measured position.
 *
 */
template< class VType, typename ForwardIterator1, typename ForwardIterator2 >
class PhantomFWKinematic
{
public:
	/** 
	 * constructor.
	 */
	PhantomFWKinematic( ForwardIterator1 iJointAnglesBegin, ForwardIterator1 iJointAnglesEnd, ForwardIterator2 iPointsBegin, VType l1, VType l2 )
		: m_iJointAnglesBegin( iJointAnglesBegin )
		, m_iJointAnglesEnd( iJointAnglesEnd )
		, m_iPointsBegin(iPointsBegin)
		, m_l1(l1)
		, m_l2(l1)
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
	 * @param input containing the parameters ( k01, k02, k03, m01, m02, m03 )
	 */
	template< class VT1, class VT2 > 
	void evaluate( VT1& result, const VT2& input ) const
	{
		namespace ublas = boost::numeric::ublas;
		unsigned i( 0 );

		VType k01 = input( 0 );
		VType k02 = input( 1 );
		VType k03 = input( 2 );
		VType m01 = input( 3 );
		VType m02 = input( 4 );
		VType m03 = input( 5 );

		ForwardIterator2 iPoints(m_iPointsBegin);
		for (ForwardIterator1 it(m_iJointAnglesBegin); it != m_iJointAnglesEnd; ++i, ++it, ++iPoints) 
		{
			VType O1 = (*it)( 0 );
			VType O2 = (*it)( 1 );
			VType O3 = (*it)( 2 );
			// calculate the distance between the measurements and the position calculated based on the corrected angles and joint lengths
			VType x = (*iPoints)(0) - (-sin(O1*k01+m01)*(m_l1*cos(O2*k02+m02)+m_l2*sin(O3*k03+m03)));
			VType y = (*iPoints)(1) - ((m_l2-m_l2*cos(O3*k03+m03)+m_l1*sin(O2*k02+m02)));
			VType z = (*iPoints)(2) - (-m_l1 + cos(O1*k01+m01)*(m_l1*cos(O2*k02+m02)+m_l2*sin(O3*k03+m03)));
			
			// store result as squared euclidean distance
			VType r = (x*x)+(y*y)+(z*z);
			OPT_LOG_TRACE( "Distance for measurement: " << i << ": " << r );
			result(i) = r;
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
		VType k01 = input( 0 );
		VType k02 = input( 1 );
		VType k03 = input( 2 );
		VType m01 = input( 3 );
		VType m02 = input( 4 );
		VType m03 = input( 5 );
		
		unsigned i( 0 );
		ForwardIterator2 iPoints(m_iPointsBegin);
		for (ForwardIterator1 it(m_iJointAnglesBegin); it != m_iJointAnglesEnd; ++i, ++it, ++iPoints)
		{
			VType O1 = (*it)( 0 );
			VType O2 = (*it)( 1 );
			VType O3 = (*it)( 2 );
			VType refx = (*iPoints)( 0 );
			VType refy = (*iPoints)( 1 );
			VType refz = (*iPoints)( 2 );
			
			J( i, 0 ) = 2*O1*sin(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03))*(m_l1 + refz - cos(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03))) + 2*O1*cos(m01 + O1*k01)*(refx + sin(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03)))*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03));
			J( i, 1 ) = 2*O2*m_l1*cos(m02 + O2*k02)*(m_l2 - refy - m_l2*cos(m03 + O3*k03) + m_l1*sin(m02 + O2*k02)) + 2*O2*m_l1*cos(m01 + O1*k01)*sin(m02 + O2*k02)*(m_l1 + refz - cos(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03))) - 2*O2*m_l1*sin(m01 + O1*k01)*sin(m02 + O2*k02)*(refx + sin(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03)));
			J( i, 2 ) = 2*O3*m_l2*sin(m03 + O3*k03)*(m_l2 - refy - m_l2*cos(m03 + O3*k03) + m_l1*sin(m02 + O2*k02)) - 2*O3*m_l2*cos(m01 + O1*k01)*cos(m03 + O3*k03)*(m_l1 + refz - cos(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03))) + 2*O3*m_l2*cos(m03 + O3*k03)*sin(m01 + O1*k01)*(refx + sin(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03)));
			J( i, 3 ) = 2*cos(m01 + O1*k01)*(refx + sin(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03)))*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03)) + 2*sin(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03))*(m_l1 + refz - cos(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03)));
			J( i, 4 ) = 2*m_l1*cos(m02 + O2*k02)*(m_l2 - refy - m_l2*cos(m03 + O3*k03) + m_l1*sin(m02 + O2*k02)) - 2*m_l1*sin(m01 + O1*k01)*sin(m02 + O2*k02)*(refx + sin(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03))) + 2*m_l1*cos(m01 + O1*k01)*sin(m02 + O2*k02)*(m_l1 + refz - cos(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03)));
			J( i, 5 ) = 2*m_l2*sin(m03 + O3*k03)*(m_l2 - refy - m_l2*cos(m03 + O3*k03) + m_l1*sin(m02 + O2*k02)) + 2*m_l2*cos(m03 + O3*k03)*sin(m01 + O1*k01)*(refx + sin(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03))) - 2*m_l2*cos(m01 + O1*k01)*cos(m03 + O3*k03)*(m_l1 + refz - cos(m01 + O1*k01)*(m_l1*cos(m02 + O2*k02) + m_l2*sin(m03 + O3*k03)));
		}
	}

	/** creates a parameter vector based on the initial guess (no correction needed) */
	template< class VT >
	void buildParameterVector( VT& v )
	{
		// set defaults for correction factors and offsets
		// k01, k02, k03, m01, m02, m03
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

		ForwardIterator2 iPoints(m_iPointsBegin);
		for (ForwardIterator1 it(m_iJointAnglesBegin); it != m_iJointAnglesEnd; ++i, ++it, ++iPoints) 
		{
			VType O1 = (*it)( 0 );
			VType O2 = (*it)( 1 );
			VType O3 = (*it)( 2 );
			// calculate the distance between the measurements and the position calculated based on the angles and joint lengths
			VType x = (*iPoints)(0) - (-sin(O1)*(m_l1*cos(O2) + m_l2*sin(O3)));
			VType y = (*iPoints)(1) - (m_l2 - m_l2*cos(O3) + m_l1*sin(O2));
			VType z = (*iPoints)(2) - (cos(O1)*(m_l1*cos(O2) + m_l2*sin(O3)) - m_l1);
			// store result as squared euclidean distance
			v(i) = (x*x)+(y*y)+(z*z);
		}
	}
	
protected:

	const ForwardIterator1 m_iJointAnglesBegin;
	const ForwardIterator1 m_iJointAnglesEnd;
	const ForwardIterator2 m_iPointsBegin;
	const VType m_l1;
	const VType m_l2;
};

} } } // namespace Ubitrack::Haptics::Function

#endif //__UBITRACK_HAPTICS_FUNCTION_PHANTOMFWKINEMATIC_H_INCLUDED__
