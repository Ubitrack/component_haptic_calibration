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
 * phantom joint length estimation component.
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */

#include <log4cpp/Category.hh>

#include <utMath/MatrixOperations.h>
#include <utHaptics/PhantomLMJointLength.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <boost/lexical_cast.hpp>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PhantomJointLengthEstimation" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Phantom Joint Length Estimation Component.
 * Given a list of measurements, containing the joint angles O1,O2,O3 and a list of measurements from the phantom, the 
 * joint length l1, l2 are calculated using curve fitting.
 *
 */
class PhantomJointLengthEstimation
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	PhantomJointLengthEstimation( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > config )
		: Dataflow::TriggerComponent( sName, config )
		, m_inAngles( "JointAngles", *this )
		, m_inPositions( "HapticPosition", *this )
		, m_outJointLengths( "Output", *this )
		, m_outOriginCalib("OriginCalib", *this )
		, m_dJoint1LengthEst( 0.13335 ) // Phantom Omni Default
		, m_dJoint2LengthEst( 0.13335 ) // Phantom Omni Defaults
		, m_iMinMeasurements( 30 ) // Recommended by Harders et al.
		, m_dOriginCalibEst( Math::Vector< double, 3 >( 0, -0.11, -0.035 ) ) // Phantom Omni Defaults from Peter Weir
    {
		config->m_DataflowAttributes.getAttributeData( "joint1LengthEst", (double &)m_dJoint1LengthEst );
		config->m_DataflowAttributes.getAttributeData( "joint2LengthEst", (double &)m_dJoint2LengthEst );
		config->m_DataflowAttributes.getAttributeData( "minMeasurements", m_iMinMeasurements );

		double calibx, caliby, calibz;
		config->m_DataflowAttributes.getAttributeData( "originCalibEstX", (double &)calibx );
		config->m_DataflowAttributes.getAttributeData( "originCalibEstY", (double &)caliby );
		config->m_DataflowAttributes.getAttributeData( "originCalibEstZ", (double &)calibz );
		m_dOriginCalibEst = Math::Vector< double, 3 > (calibx, caliby, calibz);

		
		if ( m_iMinMeasurements < 15 ) {
			LOG4CPP_ERROR( logger, "Phantom Joint Length Estimation typically needs 30+ measurements for stable results .. resetting to a minimum of 15." );
			m_iMinMeasurements = 15;
		}
		
		generateSpaceExpansionPorts( config );
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp ts )
	{
		if ( m_inAngles.get()->size() < m_iMinMeasurements )
			UBITRACK_THROW( "Not enough measurements" );

		if ( m_inAngles.get()->size() != m_inPositions.get()->size() )
			UBITRACK_THROW( "Illegal number of correspondences" );

		LOG4CPP_TRACE( logger, "Phantom Joint Length Estimation computation starts." );

		// currently set up so that it computes the offset in the wrong direction ... therefore invert offset estimate
		Math::Vector< double, 5 > result = Haptics::computePhantomLMJointLength( *m_inAngles.get(), *m_inPositions.get(), m_dJoint1LengthEst, m_dJoint2LengthEst, -m_dOriginCalibEst );

		LOG4CPP_TRACE( logger, "Phantom Joint Length Estimation computation finished." );

		Math::Vector< double, 2 > jointlengths = Math::Vector< double, 2 >(result(0), result(1));
		// currently set up so that it computes the offset in the wrong direction ...
		Math::Vector< double, 3 > origin = Math::Vector< double, 3 >(result(2), result(3), result(4));
		
		m_outJointLengths.send( Measurement::Position2D( ts, jointlengths ) );
		m_outOriginCalib.send( Measurement::Position( ts, origin ) );
    }

protected:
	/** Input port InputJointAngles of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 3 > > m_inAngles;

	/** Input port InputTrackedPosition of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 3 > > m_inPositions;

	/** Output port of the component, representing the lengths of joints as Position2D. */
	Dataflow::TriggerOutPort< Measurement::Position2D > m_outJointLengths;

	/** Output port of the component, representing the lengths of joints as Position2D. */
	Dataflow::PushSupplier< Measurement::Position > m_outOriginCalib;
	/** Minimum number of corresponding measurements */
	unsigned int m_iMinMeasurements;

	/** Joint1 length Estimation */
	double m_dJoint1LengthEst;
	
	/** Joint2 length Estimation */
	double m_dJoint2LengthEst;
	
	/** Origin Calibration Estimation */
	Math::Vector< double, 3 > m_dOriginCalibEst;

};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< PhantomJointLengthEstimation > ( "PhantomJointLengthEstimation" );
}

} } // namespace Ubitrack::Components
