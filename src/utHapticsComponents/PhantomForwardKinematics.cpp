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
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
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
	: public Dataflow::Component
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
		, m_outCorrectedFactors( "Output", *this )
		, m_dJoint1Length( 133.35 ) // Phantom Omni Defaults
		, m_dJoint2Length( 133.35 ) // Phantom Omni Defaults
    {
		config->m_DataflowAttributes.getAttributeData( "joint1Length", (double &)m_dJoint1Length );
		config->m_DataflowAttributes.getAttributeData( "joint2Length", (double &)m_dJoint2Length );
		generateSpaceExpansionPorts( config );
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp ts )
	{

		Math::Vector< 3 > joint_angles(*m_inJointAngles.get());
		Math::Vector< 3 > gimbal_angles(*m_inGimbalAngles.get());
		Math::Matrix< 3, 4 > correction_factors(*m_inCorrectionFactors.get());
		double l1 = m_dJoint1Length;
		double l2 - m_dJoint2Length;
		double O1(correction_factors( 0 , 0 )*joint_angles(0)+correction_factors( 0 , 1 ));
		double O2(correction_factors( 0 , 2 )*joint_angles(1)+correction_factors( 0 , 3 ));
		double O3(correction_factors( 1 , 0 )*joint_angles(2)+correction_factors( 1 , 1 ));
		double O4(correction_factors( 1 , 2 )*gimbal_angles(0)+correction_factors( 1 , 3 ));
		double O5(correction_factors( 2 , 0 )*gimbal_angles(1)+correction_factors( 2 , 1 ));
		double O6(correction_factors( 2 , 2 )*gimbal_angles(2)+correction_factors( 2 , 3 ));

		// calculate translation
		Math::Vector< 3 > trans(-sin(O1)*(l1*cos(O2)+l2*sin(O3)),
								l2-l2*cos(O3)+l1*sin(O2),
								-l1+cos(O1)*(l1*cos(O2)+l2*sin(O3)));

		// calculate rotation of arm (check row/column major)
		double m[9];
		m[0] = cos(O1);
		m[1] = 0.0;
		m[2] = -sin(O1);
		m[3] = -sin(O1)*sin(O3);
		m[4] = cos(-O3);
		m[5] = -cos(O1)*sin(-O3);
		m[6] = cos(O3)*sin(O1);
		m[7] = sin(-O3);
		m[8] = cos(O1)*cos(-O3);
		Math::Matrix< 3, 3> rot_arm(m);

		// the gimbal angles
		Math::Quaternion rot_gimbal1(Math::Vector< 3 >(0, 1, 0), O4);
		Math::Quaternion rot_gimbal2(Math::Vector< 3 >(1, 0, 0), -O5);
		Math::Quaternion rot_gimbal3(Math::Vector< 3 >(0, 0, 1), -O6);

		// compose rotations
		Math::Quaternion q(rot_arm*rot_gimbal1*rot_gimbal2*rot_gimbal3);

		m_outPose.send( Measurement::Pose( ts, Math::Pose( q.normalize(), trans ) ) );		
    }

protected:
	/** Input port InputJointAngles of the component. */
	Dataflow::PushConsumer< Math::Vector< 3 > > m_inJointAngles;

	/** Input port InputGimbalAngles of the component. */
	Dataflow::PushConsumer< Math::Vector< 3 > > m_inGimbalAngles;

	/** Input port Correction Factors for angles of the component. */
	Dataflow::PullConsumer< Math::Matrix< 3, 4 > > m_inCorrectionFactors;

	/** Output port of the component returns the calculated 6D pose calculated from the angles and joint lengths. */
	Dataflow::PushSupplier< Measurement::Pose > m_outPose;

	/** Joint1 length */
	double m_dJoint1Length;
	
	/** Joint2 length */
	double m_dJoint2Length;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< PhantomForwardKinematics > ( "PhantomForwardKinematics" );
}

} } // namespace Ubitrack::Components
