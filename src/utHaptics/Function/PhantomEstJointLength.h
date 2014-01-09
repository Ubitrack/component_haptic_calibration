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
 * objective function for joint length estimation using levenberg-marquardt fitting
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */

#ifndef __UBITRACK_HAPTICS_FUNCTION_PHANTOMESTJOINTLENGTH_H_INCLUDED__
#define __UBITRACK_HAPTICS_FUNCTION_PHANTOMESTJOINTLENGTH_H_INCLUDED__
 
#include <utHaptics.h>
//#include <utMath/Functors/Vector3Functors.h>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

namespace Ubitrack { namespace Haptics { namespace Function {

/**
 * Function that estimates the joint length off phantom devices.
 */
template< class VType, typename ForwardIterator1, typename ForwardIterator2 >
class PhantomEstJointLength
{
public:
	/** 
	 * constructor.
	 * @param iBegin iterator to the beginning of a contianer with projections(must stay constant during lifetime of the object)
	 * @param iEnd iterator to the end of a container with projections(must stay constant during lifetime of the object)
	 */
	PhantomEstJointLength( ForwardIterator1 iJointAnglesBegin, ForwardIterator1 iJointAnglesEnd, ForwardIterator2 iPointsBegin,
			VType l1_est, VType l2_est, const Math::Vector< VType, 3 >& calib_est)
		: m_iJointAnglesBegin( iJointAnglesBegin )
		, m_iJointAnglesEnd( iJointAnglesEnd )
		, m_iPointsBegin(iPointsBegin)
		, m_l1_est(l1_est)
		, m_l2_est(l1_est)
		, m_calib_est(calib_est)
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
		return 5;
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

		const VType l1 = input( 0 );
		const VType l2 = input( 1 );
		const VType calx = input( 2 );
		const VType caly = input( 3 );
		const VType calz = input( 4 );

		ForwardIterator2 iPoints(m_iPointsBegin);
		for (ForwardIterator1 it(m_iJointAnglesBegin); it != m_iJointAnglesEnd; ++i, ++it, ++iPoints) 
		{
			const VType O1 = (*it)( 0 );
			const VType O2 = (*it)( 1 );
			const VType O3 = (*it)( 2 );
			// calculate the distance between the measurements and the position calculated based on the joint angles and varying joint lengths
			const VType x = ((*iPoints)(0) + calx) - (-sin(O1)*(l1*cos(O2)+l2*sin(O3)));
			const VType y = ((*iPoints)(1) + caly) - ((l2-l2*cos(O3)+l1*sin(O2)));
			const VType z = ((*iPoints)(2) + calz) - (-l1 + cos(O1)*(l1*cos(O2)+l2*sin(O3)));
			
			// store result as squared euclidean distance
			result(i) = (x*x)+(y*y)+(z*z);
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
		const VType l1 = input( 0 );
		const VType l2 = input( 1 );
		const VType calx = input( 2 );
		const VType caly = input( 3 );
		const VType calz = input( 4 );
		
		unsigned i( 0 );
		ForwardIterator2 iPoints(m_iPointsBegin);
		for (ForwardIterator1 it(m_iJointAnglesBegin); it != m_iJointAnglesEnd; ++i, ++it, ++iPoints)
		{
			const VType O1 = (*it)( 0 );
			const VType O2 = (*it)( 1 );
			const VType O3 = (*it)( 2 );
			const VType refx = (*iPoints)( 0 );
			const VType refy = (*iPoints)( 1 );
			const VType refz = (*iPoints)( 2 );

			//J( i, 0 ) = 2*sin(O2)*(l2 - refy - l2*cos(O3) + l1*sin(O2)) - 2*(cos(O1)*cos(O2) - 1)*(l1 + refz - cos(O1)*(l1*cos(O2) + l2*sin(O3))) + 2*cos(O2)*sin(O1)*(refx + sin(O1)*(l1*cos(O2) + l2*sin(O3)));
			//J( i, 1 ) = 2*sin(O1)*sin(O3)*(refx + sin(O1)*(l1*cos(O2) + l2*sin(O3))) - 2*(cos(O3) - 1)*(l2 - refy - l2*cos(O3) + l1*sin(O2)) - 2*cos(O1)*sin(O3)*(l1 + refz - cos(O1)*(l1*cos(O2) + l2*sin(O3)));
			J( i, 0 ) = 2*cos(O2)*sin(O1)*(calx + refx + sin(O1)*(l1*cos(O2) + l2*sin(O3))) - 2*(cos(O1)*cos(O2) - 1)*(calz + l1 + refz - cos(O1)*(l1*cos(O2) + l2*sin(O3))) - 2*sin(O2)*(caly - l2 + refy + l2*cos(O3) - l1*sin(O2));
			J( i, 1 ) = 2*(cos(O3) - 1)*(caly - l2 + refy + l2*cos(O3) - l1*sin(O2)) + 2*sin(O1)*sin(O3)*(calx + refx + sin(O1)*(l1*cos(O2) + l2*sin(O3))) - 2*cos(O1)*sin(O3)*(calz + l1 + refz - cos(O1)*(l1*cos(O2) + l2*sin(O3)));
			J( i, 2 ) = 2*calx + 2*refx + 2*sin(O1)*(l1*cos(O2) + l2*sin(O3));
			J( i, 3 ) = 2*caly - 2*l2 + 2*refy + 2*l2*cos(O3) - 2*l1*sin(O2);
			J( i, 4 ) = 2*calz + 2*l1 + 2*refz - 2*cos(O1)*(l1*cos(O2) + l2*sin(O3));
		}
	}

	/** creates a parameter vector based on the initial guess (no correction needed) */
	template< class VT >
	void buildParameterVector( VT& v )
	{
		// set estimated lengtsh
		// l1, l2
		v( 0 ) = m_l1_est;
		v( 1 ) = m_l2_est;
		v( 2 ) = m_calib_est( 0 );
		v( 3 ) = m_calib_est( 1 );
		v( 4 ) = m_calib_est( 2 );
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
	const ForwardIterator2 m_iPointsBegin;
	const VType m_l1_est;
	const VType m_l2_est;
	const Math::Vector< VType, 3 > m_calib_est;
};

} } } // namespace Ubitrack::Haptics::Function

#endif //__UBITRACK_HAPTICS_FUNCTION_PHANTOMESTJOINTLENGTH_H_INCLUDED__
