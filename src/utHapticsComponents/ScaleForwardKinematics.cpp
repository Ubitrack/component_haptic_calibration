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

#include <utHaptics/Function/PhantomForwardKinematics.h>

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
		, m_inJACalib( "JACalib", *this )
		, m_inGACalib( "GACalib", *this )
		, m_outPose( "Output", *this )
		, m_dJointLengths( 0.20955, 0.20955 ) // Phantom Premium Defaults
		, m_dOriginCalib( Math::Vector< double, 3 >(0, 0, 0))
    {

		double joint1length, joint2length;
		config->m_DataflowAttributes.getAttributeData( "joint1Length", (double &)joint1length );
		config->m_DataflowAttributes.getAttributeData( "joint2Length", (double &)joint2length );
		m_dJointLengths = Math::Vector< double, 2 > (joint1length, joint2length);

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

		Math::Pose pose = Haptics::Function::computePhantomForwardKinematicsPose(
				*(m_inJointAngles.get()), *(m_inGimbalAngles.get()),
				*(m_inJACalib.get( ts )), *(m_inGACalib.get( ts )),
				m_dJointLengths, m_dOriginCalib);

		m_outPose.send( Measurement::Pose( ts, pose ) );
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

	/** Joint lengths */
	Math::Vector< double, 2 > m_dJointLengths;

	/** Origin Calibration */
	Math::Vector< double, 3 > m_dOriginCalib;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< PhantomForwardKinematics > ( "PhantomForwardKinematics" );
}

} } // namespace Ubitrack::Components
