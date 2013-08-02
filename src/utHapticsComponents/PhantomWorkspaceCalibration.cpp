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
 * phantom workspace calibration component.
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */
#include <log4cpp/Category.hh>

#include <utMath/MatrixOperations.h>
#include <utHaptics/PhantomLMCalibration.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <boost/lexical_cast.hpp>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomWorkspaceCalibration" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Phantom Workspace Calibration Component.
 * Given a list of measurements, containing the joint angles O1,O2,O3 and a list of measurements from a tracker, the 
 * correction factors k01, m01, k02, m02, k03, m03 are calculated using curve fitting.
 *
 * @par Operation
 * The component computes correction factors for joint angle sensors of a phantom haptic device using an external tracking system. 
 * This calibration method is based on Harders et al., Calibration, Registration, and Synchronization for High Precision Augmented Reality Haptics, 
 * IEEE Transactions on Visualization and Computer Graphics, 2009.
 *
 */
class PhantomWorkspaceCalibration
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	PhantomWorkspaceCalibration( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > config )
		: Dataflow::TriggerComponent( sName, config )
		, m_inAngles( "JointAngles", *this )
		, m_inPositions( "TrackedPosition", *this )
		, m_outCorrectedFactors( "Output", *this )
		, m_iMinMeasurements( 30 ) // Recommended by Harders et al.
		, m_dJoint1Length( 133.35 ) // Phantom Omni Defaults
		, m_dJoint2Length( 133.35 ) // Phantom Omni Defaults
		, m_dOriginCalib( Math::Vector< 3 >(0, 0, 0))
    {
		config->m_DataflowAttributes.getAttributeData( "joint1Length", (double &)m_dJoint1Length );
		config->m_DataflowAttributes.getAttributeData( "joint2Length", (double &)m_dJoint2Length );
		config->m_DataflowAttributes.getAttributeData( "minMeasurements", m_iMinMeasurements );
		double calibx, caliby, calibz;
		config->m_DataflowAttributes.getAttributeData( "originCalibX", (double &)calibx );
		config->m_DataflowAttributes.getAttributeData( "originCalibY", (double &)caliby );
		config->m_DataflowAttributes.getAttributeData( "originCalibZ", (double &)calibz );

		m_dOriginCalib = Math::Vector< 3 > (calibx, caliby, calibz);
		
		if ( m_iMinMeasurements < 15 ) {
			LOG4CPP_ERROR( logger, "Phantom Workspace Calibration typically needs 30+ measurements for stable results .. resetting to a minimum of 15." );
			m_iMinMeasurements = 15;
		}
		
		generateSpaceExpansionPorts( config );
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp ts )
	{
		if ( m_inAngles.get()->size() < m_iMinMeasurements )
			UBITRACK_THROW( "Illegal number of correspondences "  );
		if ( m_inAngles.get()->size() != m_inPositions.get()->size() )
			UBITRACK_THROW( "List length differs "  );
		LOG4CPP_INFO( logger, "call computePhantomLMCalibration:" <<  m_inAngles.get()->size());
		Math::Matrix< 3, 4 > corrFactors = Haptics::computePhantomLMCalibration( *m_inAngles.get(), *m_inPositions.get(), m_dJoint1Length, m_dJoint2Length, m_dOriginCalib );
		
		m_outCorrectedFactors.send( Measurement::Matrix3x4( ts, corrFactors ) );		
    }

protected:
	/** Input port InputJointAngles of the component. */
	Dataflow::ExpansionInPort< Math::Vector< 3 > > m_inAngles;

	/** Input port InputTrackedPosition of the component. */
	Dataflow::ExpansionInPort< Math::Vector< 3 > > m_inPositions;

	/** Output port of the component, represented as 3x4 matrix to hold 6 x offset/factor for 6 angles of the phantom. */
	Dataflow::TriggerOutPort< Measurement::Matrix3x4 > m_outCorrectedFactors;

	/** Minimum number of corresponding measurements */
	unsigned int m_iMinMeasurements;
	
	/** Joint1 length */
	double m_dJoint1Length;
	
	/** Joint2 length */
	double m_dJoint2Length;

	/** Origin Calibration */
	Math::Vector< 3, double > m_dOriginCalib;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< PhantomWorkspaceCalibration > ( "PhantomWorkspaceCalibration" );
}

} } // namespace Ubitrack::Components
