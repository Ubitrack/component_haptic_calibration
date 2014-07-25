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
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomForwardKinematics2" ) );

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
class PhantomForwardKinematics2
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	PhantomForwardKinematics2( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > config )
		: Dataflow::TriggerComponent( sName, config )
		, m_inJointAngles( "JointAngles", *this )
		, m_inGimbalAngles( "GimbalAngles", *this )
		, m_inJACalib( "JACalib", *this )
		, m_inGACalib( "GACalib", *this )
		, m_outPose( "Output", *this )
		, m_dJoint1Length( 0.13335 ) // Phantom Omni Defaults
		, m_dJoint2Length( 0.13335 ) // Phantom Omni Defaults
		, m_dOriginCalib( Math::Vector< double, 3 >(0, 0, 0))
    {
		// joint lenghts should be settable via ports as well
		config->m_DataflowAttributes.getAttributeData( "joint1Length", (double &)m_dJoint1Length );
		config->m_DataflowAttributes.getAttributeData( "joint2Length", (double &)m_dJoint2Length );

		// origin offsets should be settable via ports as well
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
		Math::Matrix< double, 3, 3 > jaCF = *(m_inJACalib.get( ts ));
		Math::Matrix< double, 3, 3 > gaCF = *(m_inGACalib.get( ts ));
		
		const double l1 = m_dJoint1Length;
		const double l2 = m_dJoint2Length;

		const double calx = m_dOriginCalib( 0 );
		const double caly = m_dOriginCalib( 1 );
		const double calz = m_dOriginCalib( 2 );

		const double j1 = jaCF( 0 , 0 );
		const double k1 = jaCF( 0 , 1 );
		const double m1 = jaCF( 0 , 2 );
		const double j2 = jaCF( 1 , 0 );
		const double k2 = jaCF( 1 , 1 );
		const double m2 = jaCF( 1 , 2 );
		const double j3 = jaCF( 2 , 0 );
		const double k3 = jaCF( 2 , 1 );
		const double m3 = jaCF( 2 , 2 );

		const double j4 = jaCF( 0 , 0 );
		const double k4 = gaCF( 0 , 1 );
		const double m4 = gaCF( 0 , 2 );
		const double j5 = jaCF( 1 , 0 );
		const double k5 = gaCF( 1 , 1 );
		const double m5 = gaCF( 1 , 2 );
		const double j6 = jaCF( 2 , 0 );
		const double k6 = gaCF( 2 , 1 );
		const double m6 = gaCF( 2 , 2 );

		const double O1(j1 * pow(joint_angles(0),2) + k1 * joint_angles(0) + m1);
		const double O2(j2 * pow(joint_angles(1),2) + k2 * joint_angles(1) + m2);
		const double O3(j3 * pow(joint_angles(2),2) + k3 * joint_angles(2) + m3);
		const double O4(j4 * pow(gimbal_angles(0),2) + k4 * gimbal_angles(0) + m4);
		const double O5(j5 * pow(gimbal_angles(1),2) + k5 * gimbal_angles(1) + m5);
		const double O6(j6 * pow(gimbal_angles(2),2) + k6 * gimbal_angles(2) + m6);

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

		// calculate translation
		Math::Vector< double, 3 > trans( calx - sO1*(cO2*l1 + l2*sO3),
								-cO3*l2 + caly + l1*sO2 + l2,
								cO1*(cO2*l1 + l2*sO3) + calz - l1 );

		// calculate rotation of stylus (6DOF)
		double m[9];

		// sympy 6DOF rotation
		m[0] =  cO6*(cO1*cO4 + cO3*sO1*sO4) + sO6*(-cO5*sO1*sO3 - sO5*(-cO1*sO4 + cO3*cO4*sO1));
		m[1] =  cO6*(cO5*sO1*sO3 + sO5*(-cO1*sO4 + cO3*cO4*sO1)) + sO6*(cO1*cO4 + cO3*sO1*sO4);
		m[2] =  cO5*(cO1*sO4 - cO3*cO4*sO1) + sO1*sO3*sO5;
		m[3] =  -cO6*sO3*sO4 + sO6*(-cO3*cO5 + cO4*sO3*sO5);
		m[4] =  cO6*(cO3*cO5 - cO4*sO3*sO5) - sO3*sO4*sO6;
		m[5] =  cO3*sO5 + cO4*cO5*sO3;
		m[6] =  cO6*(-cO1*cO3*sO4 + cO4*sO1) + sO6*(cO1*cO5*sO3 - sO5*(-cO1*cO3*cO4 - sO1*sO4));
		m[7] =  cO6*(-cO1*cO5*sO3 + sO5*(-cO1*cO3*cO4 - sO1*sO4)) + sO6*(-cO1*cO3*sO4 + cO4*sO1);
		m[8] =  -cO1*sO3*sO5 + cO5*(cO1*cO3*cO4 + sO1*sO4);

		Math::Matrix< double, 3, 3 > rot(m);
		Math::Quaternion q = Math::Quaternion(rot);
		m_outPose.send( Measurement::Pose( ts, Math::Pose( q.normalize(), trans ) ) );		
    }

protected:
	/** Input port InputJointAngles of the component. */
	Dataflow::TriggerInPort< Measurement::Vector3D > m_inJointAngles;

	/** Input port InputGimbalAngles of the component. */
	Dataflow::TriggerInPort< Measurement::Vector3D > m_inGimbalAngles;

	/** Input port Correction Factors for joint angles of the component. */
	Dataflow::PullConsumer< Measurement::Matrix3x3 > m_inJACalib;

	/** Input port Correction Factors for gimbal angles of the component. */
	Dataflow::PullConsumer< Measurement::Matrix3x3 > m_inGACalib;

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
	cf->registerComponent< PhantomForwardKinematics2 > ( "PhantomForwardKinematics2" );
}

} } // namespace Ubitrack::Components
