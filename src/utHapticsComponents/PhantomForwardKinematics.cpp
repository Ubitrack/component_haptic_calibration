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
 * @ingroup dataflow_components
 * @file
 * phantom forward kinematics component.
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */
#include <log4cpp/Category.hh>

#include <utMath/MatrixOperations.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <boost/lexical_cast.hpp>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomForwardKinematics" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Phantom Forward Kinematics Component.
 * Given a list of measurements, containing the joint angles O1,O2,O3 and a 3x4 Matrix containing correction function, the 
 * component calculates the pose of the haptic stylus (after workspace calibration).
 *
 * @par Operation
 * The component uses correction factors for joint angle sensors of a phantom haptic device to calculate the corrected 6D-Pose. 
 * The calibration method is based on Harders et al., Calibration, Registration, and Synchronization for High Precision Augmented Reality Haptics, 
 * IEEE Transactions on Visualization and Computer Graphics, 2009.
 *
 */
class PhantomForwardKinematics
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	PhantomForwardKinematics( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > config )
		: Dataflow::TriggerComponent( sName, config )
		, m_inJointAngles( "JointAngles", *this )
		, m_inGimbalAngles( "GimbalAngles", *this )
		, m_inCorrectionFactors( "CorrectionFactors", *this )
		, m_outPose( "Output", *this )
		, m_dJoint1Length( 133.35 ) // Phantom Omni Defaults
		, m_dJoint2Length( 133.35 ) // Phantom Omni Defaults
		, m_dOriginCalib( Math::Vector< double, 3 >(0, 0, 0))
    {
		config->m_DataflowAttributes.getAttributeData( "joint1Length", (double &)m_dJoint1Length );
		config->m_DataflowAttributes.getAttributeData( "joint2Length", (double &)m_dJoint2Length );

		double calibx, caliby, calibz;
		config->m_DataflowAttributes.getAttributeData( "originCalibX", (double &)calibx );
		config->m_DataflowAttributes.getAttributeData( "originCalibY", (double &)caliby );
		config->m_DataflowAttributes.getAttributeData( "originCalibZ", (double &)calibz );

		m_dOriginCalib = Math::Vector< double, 3 > (calibx, caliby, calibz);

		generateSpaceExpansionPorts( config );
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp ts )
	{

		Math::Vector< double, 3 > joint_angles = *(m_inJointAngles.get());
		Math::Vector< double, 3 > gimbal_angles = *(m_inGimbalAngles.get());
		Math::Matrix< double, 3, 4 > correction_factors = *(m_inCorrectionFactors.get( ts ));
		
		const double l1 = m_dJoint1Length;
		const double l2 = m_dJoint2Length;

		const double calx = m_dOriginCalib( 0 );
		const double caly = m_dOriginCalib( 1 );
		const double calz = m_dOriginCalib( 2 );

		const double k1 = correction_factors( 0 , 0 );
		const double m1 = correction_factors( 0 , 1 );
		const double k2 = correction_factors( 0 , 2 );
		const double m2 = correction_factors( 0 , 3 );
		const double k3 = correction_factors( 1 , 0 );
		const double m3 = correction_factors( 1 , 1 );
		const double k4 = correction_factors( 1 , 2 );
		const double m4 = correction_factors( 1 , 3 );
		const double k5 = correction_factors( 2 , 0 );
		const double m5 = correction_factors( 2 , 1 );
		const double k6 = correction_factors( 2 , 2 );
		const double m6 = correction_factors( 2 , 3 );

		const double O1(joint_angles(0));
		const double O2(joint_angles(1));
		const double O3(joint_angles(2));
		const double O4(gimbal_angles(0));
		const double O5(gimbal_angles(1));
		const double O6(gimbal_angles(2));

		// calculate translation
		Math::Vector< double, 3 > trans( -sin(m1 + O1*k1)*(l1*cos(m2 + O2*k2) + l2*sin(m3 + O3*k3)) + calx,
								l2 - l2*cos(m3 + O3*k3) + l1*sin(m2 + O2*k2) + caly,
								cos(m1 + O1*k1)*(l1*cos(m2 + O2*k2) + l2*sin(m3 + O3*k3)) - l1 + calz );

		// calculate rotation of stylus (6DOF)
		double m[9];

		// MATLAB 5DOF rotation
		//m[0] = cos(m1 + O1*k1)*cos(m4 + O4*k4) - cos(m3 + O3*k3)*sin(m1 + O1*k1)*sin(m4 + O4*k4);
		//m[1] = -sin(m5 + O5*k5)*(cos(m1 + O1*k1)*sin(m4 + O4*k4) + cos(m3 + O3*k3)*cos(m4 + O4*k4)*sin(m1 + O1*k1)) - cos(m5 + O5*k5)*sin(m1 + O1*k1)*sin(m3 + O3*k3);
		//m[2] = cos(m5 + O5*k5)*(cos(m1 + O1*k1)*sin(m4 + O4*k4) + cos(m3 + O3*k3)*cos(m4 + O4*k4)*sin(m1 + O1*k1)) - sin(m1 + O1*k1)*sin(m3 + O3*k3)*sin(m5 + O5*k5);
		//m[3] = -sin(m3 + O3*k3)*sin(m4 + O4*k4);
		//m[4] = cos(m3 + O3*k3)*cos(m5 + O5*k5) - cos(m4 + O4*k4)*sin(m3 + O3*k3)*sin(m5 + O5*k5);
		//m[5] = cos(m3 + O3*k3)*sin(m5 + O5*k5) + cos(m4 + O4*k4)*cos(m5 + O5*k5)*sin(m3 + O3*k3);
		//m[6] = -cos(m4 + O4*k4)*sin(m1 + O1*k1) - cos(m1 + O1*k1)*cos(m3 + O3*k3)*sin(m4 + O4*k4);
		//m[7] = sin(m5 + O5*k5)*(sin(m1 + O1*k1)*sin(m4 + O4*k4) - cos(m1 + O1*k1)*cos(m3 + O3*k3)*cos(m4 + O4*k4)) - cos(m1 + O1*k1)*cos(m5 + O5*k5)*sin(m3 + O3*k3);
		//m[8] = -cos(m5 + O5*k5)*(sin(m1 + O1*k1)*sin(m4 + O4*k4) - cos(m1 + O1*k1)*cos(m3 + O3*k3)*cos(m4 + O4*k4)) - cos(m1 + O1*k1)*sin(m3 + O3*k3)*sin(m5 + O5*k5);

		// sympy 6DOF rotation
		m[0] = -(-(sin(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4) + sin(O4*k4 + m4)*cos(O1*k1 + m1))*sin(O5*k5 + m5) - sin(O1*k1 + m1)*sin(O3*k3 + m3)*cos(O5*k5 + m5))*sin(O6*k6 + m6) + (-sin(O1*k1 + m1)*sin(O4*k4 + m4)*cos(O3*k3 + m3) + cos(O1*k1 + m1)*cos(O4*k4 + m4))*cos(O6*k6 + m6);
		m[1] = (-(sin(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4) + sin(O4*k4 + m4)*cos(O1*k1 + m1))*sin(O5*k5 + m5) - sin(O1*k1 + m1)*sin(O3*k3 + m3)*cos(O5*k5 + m5))*cos(O6*k6 + m6) + (-sin(O1*k1 + m1)*sin(O4*k4 + m4)*cos(O3*k3 + m3) + cos(O1*k1 + m1)*cos(O4*k4 + m4))*sin(O6*k6 + m6);
		m[2] = (sin(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4) + sin(O4*k4 + m4)*cos(O1*k1 + m1))*cos(O5*k5 + m5) - sin(O1*k1 + m1)*sin(O3*k3 + m3)*sin(O5*k5 + m5);
		m[3] = -(-sin(O3*k3 + m3)*sin(O5*k5 + m5)*cos(O4*k4 + m4) + cos(O3*k3 + m3)*cos(O5*k5 + m5))*sin(O6*k6 + m6) - sin(O3*k3 + m3)*sin(O4*k4 + m4)*cos(O6*k6 + m6);
		m[4] = (-sin(O3*k3 + m3)*sin(O5*k5 + m5)*cos(O4*k4 + m4) + cos(O3*k3 + m3)*cos(O5*k5 + m5))*cos(O6*k6 + m6) - sin(O3*k3 + m3)*sin(O4*k4 + m4)*sin(O6*k6 + m6);
		m[5] = sin(O3*k3 + m3)*cos(O4*k4 + m4)*cos(O5*k5 + m5) + sin(O5*k5 + m5)*cos(O3*k3 + m3);
		m[6] = -(-(-sin(O1*k1 + m1)*sin(O4*k4 + m4) + cos(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4))*sin(O5*k5 + m5) - sin(O3*k3 + m3)*cos(O1*k1 + m1)*cos(O5*k5 + m5))*sin(O6*k6 + m6) + (-sin(O1*k1 + m1)*cos(O4*k4 + m4) - sin(O4*k4 + m4)*cos(O1*k1 + m1)*cos(O3*k3 + m3))*cos(O6*k6 + m6);
		m[7] = (-(-sin(O1*k1 + m1)*sin(O4*k4 + m4) + cos(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4))*sin(O5*k5 + m5) - sin(O3*k3 + m3)*cos(O1*k1 + m1)*cos(O5*k5 + m5))*cos(O6*k6 + m6) + (-sin(O1*k1 + m1)*cos(O4*k4 + m4) - sin(O4*k4 + m4)*cos(O1*k1 + m1)*cos(O3*k3 + m3))*sin(O6*k6 + m6);
		m[8] = (-sin(O1*k1 + m1)*sin(O4*k4 + m4) + cos(O1*k1 + m1)*cos(O3*k3 + m3)*cos(O4*k4 + m4))*cos(O5*k5 + m5) - sin(O3*k3 + m3)*sin(O5*k5 + m5)*cos(O1*k1 + m1);

		Math::Matrix< double, 3, 3 > rot(m);
		Math::Quaternion q = Math::Quaternion(rot);
		m_outPose.send( Measurement::Pose( ts, Math::Pose( q.normalize(), trans ) ) );		
    }

protected:
	/** Input port InputJointAngles of the component. */
	Dataflow::TriggerInPort< Measurement::Vector3D > m_inJointAngles;

	/** Input port InputGimbalAngles of the component. */
	Dataflow::TriggerInPort< Measurement::Vector3D > m_inGimbalAngles;

	/** Input port Correction Factors for angles of the component. */
	Dataflow::PullConsumer< Measurement::Matrix3x4 > m_inCorrectionFactors;

	/** Output port of the component returns the calculated 6D pose calculated from the angles and joint lengths. */
	Dataflow::TriggerOutPort< Measurement::Pose > m_outPose;

	/** Joint1 length */
	double m_dJoint1Length;
	
	/** Joint2 length */
	double m_dJoint2Length;

	/** Origin Calibration */
	Math::Vector< double, 3 > m_dOriginCalib;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< PhantomForwardKinematics > ( "PhantomForwardKinematics" );
}

} } // namespace Ubitrack::Components
